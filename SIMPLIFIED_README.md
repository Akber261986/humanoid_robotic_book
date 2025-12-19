# Humanoid Robotics Book - Simplified API

This is a simplified backend system with just two endpoints for the humanoid robotics book.

## Endpoints

### 1. Trigger Index (`POST /trigger-index`)
This endpoint triggers the script that processes book markdown files.

```
POST /trigger-index
```

### 2. Query Book (`GET /query-book`)
This endpoint allows you to query about the book content.

```
GET /query-book?query=your_search_query
```

## Setup

1. Install dependencies:
```bash
cd backend
pip install -r requirements.txt
```

2. Run the server:
```bash
cd backend
python main.py
```

Or use the run script:
```bash
cd backend
./run.sh
```

## Book Content

All book content is stored in the `docs/` directory organized by modules:
- `docs/module1/` - Introduction and fundamentals
- `docs/module2/` - Simulation environments
- `docs/module3/` - Vision-language-action systems
- `docs/module4/` - Deployment and real-world applications

## Usage

1. After starting the server, you can trigger the book processing:
```
POST http://localhost:8000/trigger-index
```

2. Query the book content:
```
GET http://localhost:8000/query-book?query=inverse%20kinematics
```

The system performs simple keyword matching in the markdown files to find relevant content.