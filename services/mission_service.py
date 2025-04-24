from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
from models import entities, schemas

"""mission CRUD service"""

def create_mission_with_wps(
        db: Session, 
        waypoints: List[schemas.WaypointRequest]
) -> entities.Mission:
    mission = entities.Mission
    db.add(mission)
    db.flush()

    for wp in waypoints:
        waypoint = entities.Waypoint(
            mission_id=mission.id,
            lat=wp.lat,
            lon=wp.lon,
            alt=wp.alt,
            seq=wp.seq
        )
        db.add(waypoint)
    
    db.commit()
    db.refresh(mission)
    return mission

def get_mission(db: Session, mission_id: int) -> Optional[entities.Mission]:
    return db.query(entities.Mission).filter(entities.Mission.id == mission_id).first()

def get_missions(db: Session, skip: int = 0, limit: int = 100) -> List[entities.Mission]:
    return db.query(entities.Mission)\
             .order_by(entities.Mission.created_at.desc())\
             .offset(skip)\
             .limit(limit)\
             .all()

def update_missions(
        db: Session, 
        mission_id: int, 
        waypoints: List[schemas.WaypointRequest]
) -> Optional[entities.Mission]:
    pass

def delete_mission(db: Session, mission_id: int) -> bool:
    pass
