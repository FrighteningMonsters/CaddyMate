# CaddyMate GUI

A touch-friendly GUI for a 7-inch display designed for elderly users.

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Running the Application

1. Start the Flask server:
```bash
python server.py
```

2. Open your browser and navigate to:
```
http://localhost:5000
```

The server will:
- Serve the HTML pages
- Provide API endpoints for categories and items from the SQLite database

## Pages

- **home.html** - Main menu with Browse Categories and Search Items buttons
- **browse-categories.html** - Displays all product categories from the database
- **search.html** - Search for items (to be implemented)

## API Endpoints

- `GET /api/categories` - Get all categories
- `GET /api/items/<category_id>` - Get items for a specific category

## Database

The application uses SQLite database located at `data/caddymate_store.db` with the following structure:

### Tables:
- **categories** (id, name)
- **items** (id, name, category_id, ...)
