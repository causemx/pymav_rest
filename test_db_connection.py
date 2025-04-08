import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT

def test_postgres_connection():
    """Test connection to PostgreSQL and create database if it doesn't exist"""
    
    # Connect to PostgreSQL server
    try:
        conn = psycopg2.connect(
            dbname="gcs",
            user="causemx",  # Replace with your username
            password="3308",  # Replace with your password
            host="localhost"
        )
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
        cursor = conn.cursor()
        
        print("Successfully connected to PostgreSQL server")
        
        # Check if our database exists
        cursor.execute("SELECT 1 FROM pg_database WHERE datname='gcs'")
        exists = cursor.fetchone()
        
        if not exists:
            print("Creating 'gcs' database...")
            cursor.execute("CREATE DATABASE gcs")
            print("Database 'gcs' created successfully")
        else:
            print("Database 'gcs' already exists")
        
        cursor.close()
        conn.close()
        
        # Test connection to our database
        conn = psycopg2.connect(
            dbname="gcs",
            user="causemx",  # Replace with your username
            password="3308",  # Replace with your password
            host="localhost"
        )
        print("Successfully connected to 'gcs' database")
        conn.close()
        
        return True
        
    except Exception as e:
        print(f"Database connection error: {e}")
        return False

if __name__ == "__main__":
    test_postgres_connection()