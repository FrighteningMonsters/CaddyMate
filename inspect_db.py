import sqlite3

conn = sqlite3.connect('data/caddymate_store.db')
cursor = conn.cursor()

# Get all tables
cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
tables = cursor.fetchall()
print("Tables:", tables)

# Get categories
if any('categories' in str(t).lower() for t in tables):
    cursor.execute("SELECT * FROM categories LIMIT 5")
    print("\nCategories:", cursor.fetchall())
    
    # Get column names
    cursor.execute("PRAGMA table_info(categories)")
    print("\nCategories columns:", cursor.fetchall())

conn.close()
