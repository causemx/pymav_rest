import time
from fastapi import HTTPException
from pymavlink import mavutil
from loguru import logger
import config

def get_mavlink_connection(connection_id: str = None):
    """
    Get a MAVLink connection by ID or the current active connection
    """
    # If no connection_id provided, use current connection
    if connection_id is None:
        connection_id = config.current_connection_id
    
    if connection_id is None or connection_id not in config.mavlink_connections:
        raise HTTPException(status_code=503, detail="No active MAVLink connection")
    
    connection_data = config.mavlink_connections[connection_id]
    if not connection_data["status"]["connected"]:
        raise HTTPException(status_code=503, detail=f"MAVLink connection {connection_id} is not established")
    
    return connection_data["connection"]

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