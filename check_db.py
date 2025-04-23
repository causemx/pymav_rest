#!/usr/bin/env python3
"""
Simple script to check PostgreSQL database connection
"""
import os
import psycopg2
from psycopg2 import OperationalError
from dotenv import load_dotenv

load_dotenv()

user = os.getenv("DB_USER")
password = os.getenv("DB_PASSWORD")
host = os.getenv("DB_HOST")
port = os.getenv("DB_PORT")
db_name = os.getenv("DB_NAME")


def check_db_connection():
    """Check if database connection is successful"""
    # Connection parameters - same as in database.py
    
    try:
        # Create connection
        connection = psycopg2.connect(
            user=user,
            password=password,
            host=host,
            port=port,
            database=db_name
        )
        
        # Check if connection is open
        if connection.closed == 0:
            print("Database connection successful!")
            connection.close()
            return True
        else:
            print("Connection created but immediately closed.")
            return False
            
    except OperationalError as e:
        print(f"Error connecting to PostgreSQL: {e}")
        return False


if __name__ == "__main__":
    check_db_connection()