import time
from fastapi import APIRouter, HTTPException, Depends
from pymavlink import mavutil
from loguru import logger
from sqlalchemy.orm import Session

from models.schemas import MissionRequest, MissionSummary
from models.entities import Mission, Waypoint
from utils.mavlink_helpers import get_mavlink_connection, clear_mission, get_capabilities
from database import get_db
from services.mission_service import get_missions

router = APIRouter(prefix="/mission")

@router.post("/upload")
async def upload_mission(
    mission: MissionRequest,
    mavlink_conn = Depends(get_mavlink_connection),
    db: Session = Depends(get_db),    
):
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
        conn_status = mavlink_conn.target_system is not None
        if not conn_status:
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
        
        # Save accepted mission to db
        mission_item = Mission()
        db.add(mission_item)
        db.flush()

        for wp in waypoints:
            waypoint_item = Waypoint(
                mission_id=mission_item.id,
                lat=wp.lat,
                lon=wp.lon,
                alt=wp.alt,
                seq=wp.seq
            )
            db.add(waypoint_item)
        db.commit()
        db.refresh(mission_item)
        
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

@router.get("/current")
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

@router.post("/start")
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

@router.get("/history")
async def get_mission_history(db: Session = Depends(get_db)):
    # Get all mission history from db
    missions = get_missions(db)
    # Convert to response model with waypoint count
    result = []
    for mission in missions:
        waypoint_count = len(mission.waypoints)
        result.append(
            MissionSummary(
                id=mission.id,
                waypoint_count=waypoint_count,
                created_at=mission.created_at,
                updated_at=mission.updated_at
            )
        )
    
    return result