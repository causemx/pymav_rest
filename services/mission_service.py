import schemas
from sqlalchemy.orm import Session
from typing import List, Optional
import models

"""mission CRUD service"""

def create_mission_with_wps(
        db: Session, 
        waypoints: List[schemas.WaypointRequest]
) -> models.Mission:
    mission = models.Mission()
    db.add(mission)
    db.flush()

    for wp in waypoints:
        waypoint = models.Waypoint(
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

def get_mission(db: Session, mission_id: int) -> Optional[models.Mission]:
    return db.query(models.Mission).filter(models.Mission.id == mission_id).first()

def get_missions(db: Session, skip: int = 0, limit: int = 100) -> List[models.Mission]:
    return db.query(models.Mission)\
             .order_by(models.Mission.created_at.desc())\
             .offset(skip)\
             .limit(limit)\
             .all()

def update_missions(
        db: Session, 
        mission_id: int, 
        waypoints: List[schemas.WaypointRequest]
) -> Optional[models.Mission]:
    pass

def delete_mission(db: Session, mission_id: int) -> bool:
    pass
