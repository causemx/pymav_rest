from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
from models import entities, schemas

"""mission CRUD service"""

def create_mission(db: Session, mission: entities.Mission) -> entities.Mission:
    mission_item = entities.Mission(**mission.dict())
    db.add(mission_item)
    db.commit()
    db.refresh(mission_item)
    return mission_item

def get_mission(db: Session, mission_id: int) -> Optional[entities.Mission]:
    return db.query(entities.Mission).filter(entities.Mission.id == mission_id).first()

def get_missions(db: Session):
    return db.query(entities.Mission).all()
