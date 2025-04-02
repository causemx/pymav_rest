import asyncio
import time
import uuid
import uvicorn
from fastapi import FastAPI, HTTPException, Query, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Dict
from pymavlink import mavutil
from enum import Enum
from loguru import logger

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

# Connections registry to store multiple connections
mavlink_connections = {}
current_connection_id = None

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
    name: Optional[str] = Field(None, description="User-friendly name for this connection")

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

class ConnectionStatus(BaseModel):
    id: str
    name: str
    connection_type: ConnectionType
    device: str
    connected: bool
    vehicle_type: Optional[str] = None
    system_id: Optional[int] = None
    last_heartbeat: Optional[float] = None
    created_at: float

# Utility functions
def get_mavlink_connection(connection_id: str = None):
    global mavlink_connections, current_connection_id
    
    # If no connection_id provided, use current connection
    if connection_id is None:
        connection_id = current_connection_id
    
    if connection_id is None or connection_id not in mavlink_connections:
        raise HTTPException(status_code=503, detail="No active MAVLink connection")
    
    connection_data = mavlink_connections[connection_id]
    if not connection_data["status"]["connected"]:
        raise HTTPException(status_code=503, detail=f"MAVLink connection {connection_id} is not established")
    
    return connection_data["connection"]

async def update_connection_status(connection_id):
    global mavlink_connections
    
    connection_data = mavlink_connections.get(connection_id)
    if not connection_data:
        return
    
    mavlink_connection = connection_data["connection"]
    status = connection_data["status"]
    
    while status["connected"]:
        try:
            # Try to get heartbeat
            msg = mavlink_connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                status["last_heartbeat"] = time.time()
                if not status["vehicle_type"]:
                    status["vehicle_type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name
                if not status["system_id"]:
                    status["system_id"] = mavlink_connection.target_system
            
            # Check if connection is still alive
            if status["last_heartbeat"] and time.time() - status["last_heartbeat"] > 5:
                status["connected"] = False
                break
                
        except Exception as e:
            logger.error(f"Error updating connection status for {connection_id}: {e}")
            status["connected"] = False
            break
            
        await asyncio.sleep(1)

# API Endpoints
@app.get("/")
async def root():
    return {"message": "MAVLink API Server is running!"}

@app.get("/status")
async def get_status(connection_id: Optional[str] = None):
    if connection_id:
        if connection_id not in mavlink_connections:
            raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
        return mavlink_connections[connection_id]["status"]
    
    global current_connection_id
    if current_connection_id:
        return mavlink_connections[current_connection_id]["status"]
    
    raise HTTPException(status_code=503, detail="No active MAVLink connection")

@app.get("/connections")
async def list_connections():
    """
    List all MAVLink connections.
    Returns details about all vehicle connections including connection status, type, and vehicle info.
    """
    connection_list = []
    
    for conn_id, conn_data in mavlink_connections.items():
        status = conn_data["status"]
        connection_list.append({
            "id": conn_id,
            "name": status["name"],
            "connection_type": status["connection_type"],
            "device": status["device"],
            "connected": status["connected"],
            "vehicle_type": status["vehicle_type"],
            "system_id": status["system_id"],
            "last_heartbeat": status["last_heartbeat"],
            "is_current": conn_id == current_connection_id,
            "created_at": status["created_at"]
        })
    
    return {
        "total": len(connection_list),
        "connections": connection_list,
        "current_connection": current_connection_id
    }

@app.post("/connect")
async def connect(connection_req: ConnectionRequest):
    global mavlink_connections, current_connection_id
    
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
        
        # Generate unique ID for this connection
        connection_id = str(uuid.uuid4())
        
        # Create connection name if not provided
        connection_name = connection_req.name
        if not connection_name:
            connection_name = f"{connection_req.connection_type.value}-{connection_req.device}"
        
        # Create connection status
        connection_status = {
            "id": connection_id,
            "name": connection_name,
            "connection_type": connection_req.connection_type,
            "device": connection_req.device,
            "connected": True,
            "vehicle_type": mavutil.mavlink.enums['MAV_TYPE'][msg.type].name,
            "system_id": mavlink_connection.target_system,
            "last_heartbeat": time.time(),
            "created_at": time.time()
        }
        
        # Store connection in registry
        mavlink_connections[connection_id] = {
            "connection": mavlink_connection,
            "status": connection_status
        }
        
        # Set as current connection
        current_connection_id = connection_id
        
        # Start background task to update connection status
        asyncio.create_task(update_connection_status(connection_id))
        
        return {
            "status": "connected", 
            "connection_id": connection_id,
            "details": connection_status
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Connection failed: {str(e)}")

@app.post("/set_active_connection/{connection_id}")
async def set_active_connection(connection_id: str):
    global mavlink_connections, current_connection_id
    
    if connection_id not in mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    if not mavlink_connections[connection_id]["status"]["connected"]:
        raise HTTPException(status_code=400, detail=f"Connection {connection_id} is not active")
    
    current_connection_id = connection_id
    
    return {
        "status": "success",
        "message": f"Connection {connection_id} is now active",
        "connection": mavlink_connections[connection_id]["status"]
    }

@app.post("/disconnect")
async def disconnect(connection_id: Optional[str] = None):
    global mavlink_connections, current_connection_id
    
    # If no connection_id provided, use current connection
    if connection_id is None:
        connection_id = current_connection_id
    
    if connection_id is None:
        raise HTTPException(status_code=400, detail="No connection specified")
    
    if connection_id not in mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    connection_data = mavlink_connections[connection_id]
    connection_data["connection"].close()
    connection_data["status"]["connected"] = False
    del mavlink_connections[connection_id]

    # If disconnecting current connection, set current to None
    if connection_id == current_connection_id:
        current_connection_id = None
        
        # Try to set another active connection as current
        for conn_id, conn_data in mavlink_connections.items():
            if conn_id != connection_id and conn_data["status"]["connected"]:
                current_connection_id = conn_id
                break
    
    return {
        "status": "disconnected",
        "connection_id": connection_id,
        "current_connection": current_connection_id
    }

@app.delete("/connection/{connection_id}")
async def delete_connection(connection_id: str):
    global mavlink_connections, current_connection_id
    
    if connection_id not in mavlink_connections:
        raise HTTPException(status_code=404, detail=f"Connection {connection_id} not found")
    
    # Disconnect if connected
    connection_data = mavlink_connections[connection_id]
    if connection_data["status"]["connected"]:
        connection_data["connection"].close()
    
    # Remove from registry
    del mavlink_connections[connection_id]
    
    # If deleting current connection, set current to None
    if connection_id == current_connection_id:
        current_connection_id = None
        
        # Try to set another active connection as current
        for conn_id, conn_data in mavlink_connections.items():
            if conn_data["status"]["connected"]:
                current_connection_id = conn_id
                break
    
    return {
        "status": "deleted",
        "connection_id": connection_id,
        "current_connection": current_connection_id
    }

@app.get("/vehicle/parameters")
async def get_parameters(connection_id: Optional[str] = None, 
                         mavlink_conn = Depends(get_mavlink_connection)):
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
    """
    Upload a mission to the vehicle with enhanced error handling and recovery mechanisms.
    
    This implementation includes:
    - Capability checks
    - Partial update recovery
    - Progress tracking
    - Multiple retry attempts
    - Detailed error reporting
    """
   
    try:
        # Check if vehicle is connected
        if not connection_status["connected"] or not connection_status["last_heartbeat"]:
            raise HTTPException(status_code=400, detail="Vehicle connection unstable")
        
        # Check vehicle capabilities
        capabilities = await get_capabilities(mavlink_conn)
        use_mission_int = capabilities.get("MISSION_INT", False)
        
        # Clear existing mission if requested
        if mission.clear_existing:
            clear_result = await clear_mission(mavlink_conn)
            if not clear_result["success"]:
                raise HTTPException(status_code=400, detail=f"Failed to clear mission: {clear_result['message']}")
        
        # Prepare mission items
        waypoints = mission.waypoints
        if not waypoints:
            raise HTTPException(status_code=400, detail="No waypoints provided")
        
        # Start mission upload
        mavlink_conn.waypoint_count_send(len(waypoints))
        
        start_time = time.time()
        timeout = 5  # seconds
        
        current_waypoint_index = 0
        retry_count = 0
        max_retries = 3
        
        while current_waypoint_index < len(waypoints) and retry_count < max_retries:
            
            msg = mavlink_conn.recv_match(type='MISSION_REQUEST', blocking=True, timeout=timeout)
            
            if not msg:
                
                if retry_count < max_retries:
                    
                    mavlink_conn.mav.mission_write_partial_list_send(
                        mavlink_conn.target_system,
                        mavlink_conn.target_component,
                        current_waypoint_index,  # Start index
                        len(waypoints)  # End index
                    )
                    retry_count += 1
                    continue
                else:
                    raise HTTPException(status_code=504, detail=f"Timeout waiting for MISSION_REQUEST for waypoint {current_waypoint_index}")
            
            # Reset retry counter on successful request
            retry_count = 0
            
            # Check if the request is for the waypoint we expect
            if msg.seq != current_waypoint_index:
                # Handle sequence mismatch (like in C# implementation)
                current_waypoint_index = msg.seq
                # Log this mismatch
                logger.info(f"Sequence mismatch: expected {current_waypoint_index}, got {msg.seq}")            
            # Get the waypoint
            waypoint = waypoints[current_waypoint_index]
            
            # Record progress
            logger.info(f"Uploading waypoint {current_waypoint_index + 1}/{len(waypoints)}")
            
            # Send the requested waypoint
            if use_mission_int:
                # Use MISSION_ITEM_INT format if supported
                mavlink_conn.mav.mission_item_int_send(
                    mavlink_conn.target_system,
                    mavlink_conn.target_component,
                    current_waypoint_index,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0,  # Current, autocontinue
                    0, 0, 0, 0,  # Params (hold time, acceptance radius, pass radius, yaw)
                    int(waypoint.lat * 10000000),  # Latitude as int32 (deg * 10^7)
                    int(waypoint.lon * 10000000),  # Longitude as int32 (deg * 10^7)
                    waypoint.alt
                )
            else:
                # Use standard MISSION_ITEM format
                mavlink_conn.mav.mission_item_send(
                    mavlink_conn.target_system,
                    mavlink_conn.target_component,
                    current_waypoint_index,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 0,  # Current, autocontinue
                    0, 0, 0, 0,  # Params (hold time, acceptance radius, pass radius, yaw)
                    waypoint.lat,
                    waypoint.lon,
                    waypoint.alt
                )
            
            # Move to next waypoint
            current_waypoint_index += 1
            
        # Verify all waypoints uploaded
        if current_waypoint_index < len(waypoints):
            raise HTTPException(status_code=400, detail=f"Mission upload incomplete: only {current_waypoint_index}/{len(waypoints)} waypoints uploaded")
        
        # Wait for final acknowledgment
        ack = mavlink_conn.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if not ack:
            raise HTTPException(status_code=504, detail="No mission acknowledgment received after upload")
        
        # Check result
        if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise HTTPException(status_code=400, detail="Mission upload failed")
        
        # Success
        return {
            "status": "mission uploaded",
            "waypoints_count": len(waypoints),
            "elapsed_time": round(time.time() - start_time, 2)
        }
        
    except Exception as e:
        if isinstance(e, HTTPException):
            raise
        raise HTTPException(status_code=500, detail=f"Mission upload failed: {str(e)}")

# Helper functions

async def clear_mission(mavlink_conn):
    """
    Clear the current mission with timeout and retry logic
    """
    retry_count = 0
    max_retries = 3
    
    while retry_count < max_retries:
        try:
            mavlink_conn.waypoint_clear_all_send()
            ack = mavlink_conn.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
            
            if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return {"success": True, "message": "Mission cleared successfully"}
            elif ack:
                result = f"Clear mission failed with code: {ack.type}"
                retry_count += 1
            else:
                result = "No acknowledgment received"
                retry_count += 1
                
        except Exception as e:
            result = str(e)
            retry_count += 1
    
    return {"success": False, "message": result}

async def get_capabilities(mavlink_conn):
    """
    Get vehicle capabilities from AUTOPILOT_VERSION message
    """
    capabilities = {}
    
    # Request capabilities
    mavlink_conn.mav.command_long_send(
        mavlink_conn.target_system,
        mavlink_conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,  # Confirmation
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for response
    msg = mavlink_conn.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
    
    if msg:
        # Check specific capabilities
        cap_flags = msg.capabilities
        capabilities["MISSION_FLOAT"] = bool(cap_flags & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT)
        capabilities["MISSION_INT"] = bool(cap_flags & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT)
        capabilities["MISSION_FENCE"] = bool(cap_flags & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FENCE)
        capabilities["MISSION_RALLY"] = bool(cap_flags & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_RALLY)
    else:
        # Fallback to assume basic capabilities
        capabilities["MISSION_FLOAT"] = True
        capabilities["MISSION_INT"] = False
        capabilities["MISSION_FENCE"] = False
        capabilities["MISSION_RALLY"] = False
    
    return capabilities

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