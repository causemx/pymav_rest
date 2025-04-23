from sqlalchemy.orm import Session
from typing import List, Optional, Dict, Any
from models import entities, schemas


def get_item(db: Session, item_id: int) -> Optional[entities.Item]:
    """Get item by ID"""
    return db.query(entities.Item).filter(entities.Item.id == item_id).first()


def get_items(
    db: Session, 
    skip: int = 0, 
    limit: int = 100,
    is_active: Optional[bool] = None
) -> List[entities.Item]:
    """Get list of items with pagination and optional filtering"""
    query = db.query(entities.Item)
    
    if is_active is not None:
        query = query.filter(entities.Item.is_active == is_active)
    
    return query.offset(skip).limit(limit).all()


def create_item(db: Session, item: schemas.ItemCreate) -> entities.Item:
    """Create a new item"""
    db_item = entities.Item(**item.dict())
    db.add(db_item)
    db.commit()
    db.refresh(db_item)
    return db_item
