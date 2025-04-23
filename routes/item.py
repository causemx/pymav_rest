import time
from fastapi import APIRouter, HTTPException, Query, Depends, status
from pymavlink import mavutil
from typing import List, Optional
from loguru import logger
from sqlalchemy.orm import Session

import database
from models import schemas
from services import item_service



router = APIRouter(prefix="/items")


@router.get("/", response_model=List[schemas.Item])
def read_items(
    skip: int = 0,
    limit: int = 100,
    is_active: Optional[bool] = None,
    db: Session = Depends(database.get_db)
):
    """
    Get all items with pagination and optional filtering.
    """
    items = item_service.get_items(db, skip=skip, limit=limit, is_active=is_active)
    return items


@router.get("/{item_id}", response_model=schemas.Item)
def read_item(
    item_id: int,
    db: Session = Depends(database.get_db)
):
    """
    Get an item by ID.
    """
    db_item = item_service.get_item(db, item_id=item_id)
    if db_item is None:
        raise HTTPException(status_code=404, detail="Item not found")
    return db_item


@router.post("/", response_model=schemas.Item, status_code=status.HTTP_201_CREATED)
def create_item(
    item: schemas.ItemCreate,
    db: Session = Depends(database.get_db)
):
    """
    Create a new item.
    """
    return item_service.create_item(db=db, item=item)
