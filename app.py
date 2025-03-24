import asyncio
import time
import uvicorn
from fastapi import FastAPI, HTTPException, Query, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional
from pymavlink import mavutil
from enum import Enum

app = FastAPI(
    title="MAVLink API Server",
    description="RESTful API for interacting with MAVLink-enabled vehicles",
    version="1.0.0"
)

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global connection variable
mavlink_connection = None
connection_status = {
    "connected": False,
    "vehicle_type": None,
    "system_id": None,
    "last_heartbeat": None
}

# Data models
class ConnectionType(str, Enum):
    SERIAL = "serial"
    UDP = "udp"
    TCP = "tcp"

class ConnectionRequest(BaseModel):
    connection_type: ConnectionType
    device: str = Field(..., description="Serial device, UDP address:port, or TCP address:port")
    baud: Optional[int] = Field(57600, description="Baud rate for serial connections")
    timeout: Optional[float] = Field(1.0, description="Connection timeout in seconds")

class CommandRequest(BaseModel):
    command_id: int
    params: List[float] = Field(default_factory=lambda: [0, 0, 0, 0, 0, 0, 0])
    target_system: Optional[int] = None
    target_component: Optional[int] = None
    confirmation: Optional[int] = Field(0, description="Confirmation parameter")

class WaypointRequest(BaseModel):
    lat: float = Field(..., description="Latitude in degrees")
    lon: float = Field(..., description="Longitude in degrees")
    alt: float = Field(..., description="Altitude in meters (relative to home)")
    seq: int = Field(..., description="Waypoint sequence number")

class MissionRequest(BaseModel):
    waypoints: List[WaypointRequest]
    clear_existing: bool = Field(True, description="Clear existing mission before uploading")

# Utility functions
def get_mavlink_connection():
    global mavlink_connection
    if mavlink_connection is None or not connection_status["connected"]:
        raise HTTPException(status_code=503, detail="MAVLink connection not established")
    return mavlink_connection

async def update_connection_status():
    global mavlink_connection, connection_status
    
    while connection_status["connected"]:
        try:
            # Try to get heartbeat
            msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                connection_status["last_heartbeat"] = time.time()
                if not connection_status["vehicle_type"]:
                    connection_status["vehicle_type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name
                if not connection_status["system_id"]:
                    connection_status["system_id"] = mavlink_connection.target_system
            
            # Check if connection is still alive
            if connection_status["last_heartbeat"] and time.time() - connection_status["last_heartbeat"] > 5:
                connection_status["connected"] = False
                break
                
        except Exception as e:
            print(f"Error updating connection status: {e}")
            connection_status["connected"] = False
            break
            
        await asyncio.sleep(1)

# API Endpoints
@app.get("/")
async def root():
    return {"message": "MAVLink API Server is running!"}

@app.get("/status")
async def get_status():
    return connection_status

@app.post("/connect")
async def connect(connection_req: ConnectionRequest):
    global mavlink_connection, connection_status
    
    # Close existing connection if any
    if mavlink_connection and connection_status["connected"]:
        mavlink_connection.close()
        connection_status["connected"] = False
    
    try:
        # Create connection string based on connection type
        if connection_req.connection_type == ConnectionType.SERIAL:
            conn_str = f"{connection_req.connection_type.value}:{connection_req.device}:{connection_req.baud}"
        else:
            conn_str = f"{connection_req.connection_type.value}:{connection_req.device}"
        
        # Establish connection
        mavlink_connection = mavutil.mavlink_connection(conn_str, autoreconnect=True, timeout=connection_req.timeout)
        
        # Wait for heartbeat
        msg = mavlink_connection.wait_heartbeat(timeout=5)
        if not msg:
            raise HTTPException(status_code=504, detail="No heartbeat received after 5 seconds")
        
        # Update connection status
        connection_status = {
            "connected": True,
            "vehicle_type": mavutil.mavlink.enums['MAV_TYPE'][msg.type].name,
            "system_id": mavlink_connection.target_system,
            "last_heartbeat": time.time()
        }
        
        # Start background task to update connection status
        asyncio.create_task(update_connection_status())
        
        return {"status": "connected", "details": connection_status}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Connection failed: {str(e)}")

@app.post("/disconnect")
async def disconnect():
    global mavlink_connection, connection_status
    
    if mavlink_connection:
        mavlink_connection.close()
        
    connection_status = {
        "connected": False,
        "vehicle_type": None,
        "system_id": None,
        "last_heartbeat": None
    }
    
    return {"status": "disconnected"}

@app.get("/vehicle/parameters")
async def get_parameters(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Request parameter list
        mavlink_conn.param_fetch_all()
        
        # Wait for parameters (with timeout)
        start_time = time.time()
        params = {}
        
        while time.time() - start_time < 10:
            msg = mavlink_conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if msg:
                params[msg.param_id] = msg.param_value
            else:
                break
        
        if not params:
            raise HTTPException(status_code=504, detail="No parameters received")
            
        return {"parameters": params}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get parameters: {str(e)}")

@app.get("/vehicle/attitude")
async def get_attitude(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Request attitude information
        msg = mavlink_conn.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if not msg:
            raise HTTPException(status_code=504, detail="No attitude data received")
            
        return {
            "timestamp": msg.time_boot_ms,
            "roll": msg.roll,
            "pitch": msg.pitch,
            "yaw": msg.yaw,
            "rollspeed": msg.rollspeed,
            "pitchspeed": msg.pitchspeed,
            "yawspeed": msg.yawspeed
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get attitude: {str(e)}")

@app.get("/vehicle/location")
async def get_location(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Request GPS and global position data
        gps = mavlink_conn.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
        pos = mavlink_conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        
        if not gps or not pos:
            raise HTTPException(status_code=504, detail="No location data received")
            
        return {
            "lat": pos.lat / 1e7,  # Convert from int to float degrees
            "lon": pos.lon / 1e7,
            "alt": pos.alt / 1000.0,  # Convert to meters
            "relative_alt": pos.relative_alt / 1000.0,
            "gps_fix_type": gps.fix_type,
            "gps_satellites_visible": gps.satellites_visible,
            "heading": pos.hdg / 100.0 if pos.hdg != 0 else 0,  # Convert heading to degrees
            "velocity": {
                "vx": pos.vx / 100.0,  # cm/s to m/s
                "vy": pos.vy / 100.0,
                "vz": pos.vz / 100.0
            }
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get location: {str(e)}")

@app.post("/vehicle/arm")
async def arm_vehicle(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send arm command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            1,  # Arm (1 = arm, 0 = disarm)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            raise HTTPException(status_code=504, detail="No acknowledgment received for arm command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"Arm command rejected: {result_text}")
        
        return {"status": "armed", "result": "success"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to arm vehicle: {str(e)}")

@app.post("/vehicle/disarm")
async def disarm_vehicle(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send disarm command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            0,  # Disarm (1 = arm, 0 = disarm)
            0, 0, 0, 0, 0, 0  # Unused parameters
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            raise HTTPException(status_code=504, detail="No acknowledgment received for disarm command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"Disarm command rejected: {result_text}")
        
        return {"status": "disarmed", "result": "success"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to disarm vehicle: {str(e)}")

@app.post("/vehicle/takeoff")
async def takeoff(altitude: float = Query(..., description="Takeoff altitude in meters"), 
                 mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send takeoff command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, altitude  # Parameters (last one is altitude)
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            raise HTTPException(status_code=504, detail="No acknowledgment received for takeoff command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"Takeoff command rejected: {result_text}")
        
        return {"status": "takeoff initiated", "target_altitude": altitude}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to takeoff: {str(e)}")

@app.post("/vehicle/land")
async def land(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send land command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Unused parameters
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_NAV_LAND:
            raise HTTPException(status_code=504, detail="No acknowledgment received for land command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"Land command rejected: {result_text}")
        
        return {"status": "landing initiated"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to land: {str(e)}")

@app.post("/vehicle/return_to_launch")
async def return_to_launch(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send RTL command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Unused parameters
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            raise HTTPException(status_code=504, detail="No acknowledgment received for RTL command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"RTL command rejected: {result_text}")
        
        return {"status": "return to launch initiated"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to initiate RTL: {str(e)}")

@app.post("/vehicle/goto")
async def goto_position(lat: float = Query(..., description="Latitude in degrees"),
                       lon: float = Query(..., description="Longitude in degrees"), 
                       alt: float = Query(..., description="Altitude in meters"),
                       mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send goto command
        mavlink_conn.mav.command_int_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # Current (0 = false, 1 = true)
            0,  # Autocontinue (0 = false, 1 = true)
            0,  # Param1: Hold time
            0,  # Param2: Accept radius
            0,  # Param3: Pass radius
            0,  # Param4: Yaw
            int(lat * 1e7),  # Latitude (degrees * 10^7)
            int(lon * 1e7),  # Longitude (degrees * 10^7)
            alt   # Altitude (in meters)
        )
        
        return {"status": "goto command sent", "target": {"lat": lat, "lon": lon, "alt": alt}}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to send goto command: {str(e)}")

@app.post("/vehicle/command")
async def send_command(command: CommandRequest, mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Get target system/component or use connection defaults
        target_system = command.target_system if command.target_system is not None else mavlink_conn.target_system
        target_component = command.target_component if command.target_component is not None else mavlink_conn.target_component
        
        # Send command
        mavlink_conn.mav.command_long_send(
            target_system,
            target_component,
            command.command_id,
            command.confirmation,
            command.params[0],
            command.params[1],
            command.params[2],
            command.params[3],
            command.params[4],
            command.params[5],
            command.params[6]
        )
        
        # Wait for acknowledgment if confirmation requested
        if command.confirmation > 0:
            msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
            if not msg or msg.command != command.command_id:
                raise HTTPException(status_code=504, detail="No acknowledgment received")
                
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            return {"command": command.command_id, "result": result_text}
        
        return {"status": "command sent", "command_id": command.command_id}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to send command: {str(e)}")

@app.post("/mission/upload")
async def upload_mission(mission: MissionRequest, mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Clear existing mission if requested
        if mission.clear_existing:
            mavlink_conn.waypoint_clear_all_send()
            ack = mavlink_conn.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise HTTPException(status_code=400, detail="Failed to clear existing mission")
        
        # Start mission upload
        mavlink_conn.waypoint_count_send(len(mission.waypoints))
        
        # Process mission items
        for waypoint in mission.waypoints:
            # Wait for request
            req = mavlink_conn.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
            if not req:
                raise HTTPException(status_code=504, detail=f"No mission request received for waypoint {waypoint.seq}")
            
            # Send the requested waypoint
            mavlink_conn.mav.mission_item_send(
                mavlink_conn.target_system,
                mavlink_conn.target_component,
                waypoint.seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0, 0,  # Current, autocontinue
                0, 0, 0, 0,  # Params (hold time, acceptance radius, pass radius, yaw)
                waypoint.lat,
                waypoint.lon,
                waypoint.alt
            )
        
        # Wait for mission acknowledgment
        ack = mavlink_conn.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack or ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise HTTPException(status_code=400, detail="Mission upload failed")
        
        return {"status": "mission uploaded", "waypoints": len(mission.waypoints)}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to upload mission: {str(e)}")

@app.get("/mission/current")
async def get_current_mission(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Request mission count
        mavlink_conn.waypoint_request_list_send()
        msg = mavlink_conn.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        
        if not msg:
            raise HTTPException(status_code=504, detail="Failed to get mission count")
            
        count = msg.count
        waypoints = []
        
        # Request each waypoint
        for i in range(count):
            mavlink_conn.waypoint_request_send(i)
            item = mavlink_conn.recv_match(type='MISSION_ITEM', blocking=True, timeout=5)
            
            if not item:
                raise HTTPException(status_code=504, detail=f"Failed to get waypoint {i}")
                
            waypoints.append({
                "seq": item.seq,
                "command": item.command,
                "lat": item.x,
                "lon": item.y,
                "alt": item.z,
                "frame": item.frame,
                "params": [item.param1, item.param2, item.param3, item.param4]
            })
        
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
            0, 0, 0, 0, 0, 0, 0, 0
        )
                
        current = mavlink_conn.recv_match(type='MISSION_CURRENT', blocking=True, timeout=2)
        current_seq = current.seq if current else None
        
        return {
            "count": count,
            "current_seq": current_seq,
            "waypoints": waypoints
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get mission: {str(e)}")

@app.post("/mission/start")
async def start_mission(mavlink_conn = Depends(get_mavlink_connection)):
    try:
        # Send mission start command
        mavlink_conn.mav.command_long_send(
            mavlink_conn.target_system,
            mavlink_conn.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Parameters
        )
        
        # Wait for command acknowledgment
        msg = mavlink_conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not msg or msg.command != mavutil.mavlink.MAV_CMD_MISSION_START:
            raise HTTPException(status_code=504, detail="No acknowledgment received for mission start command")
            
        if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            result_text = mavutil.mavlink.enums['MAV_RESULT'][msg.result].name
            raise HTTPException(status_code=400, detail=f"Mission start command rejected: {result_text}")
        
        return {"status": "mission started"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start mission: {str(e)}")

if __name__ == "__main__":
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)