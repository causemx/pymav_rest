import time
from fastapi import APIRouter, HTTPException, Query, Depends
from pymavlink import mavutil
from typing import Optional
from loguru import logger

from schemas import CommandRequest
from utils.mavlink_helpers import get_mavlink_connection

router = APIRouter(prefix="/vehicle")

@router.get("/parameters")
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

@router.get("/attitude")
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

@router.get("/location")
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

@router.post("/arm")
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

@router.post("/disarm")
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

@router.post("/takeoff")
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

@router.post("/land")
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

@router.post("/return_to_launch")
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

@router.post("/goto")
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

@router.post("/command")
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