# 🧮 ROS 2 Service Demo - Calculator Service

A simple educational package to understand **ROS 2 Services** (Request/Response communication).

---

## 📋 Table of Contents

1. [What is a Service?](#-what-is-a-service)
2. [Topics vs Services](#-topics-vs-services)
3. [How This Demo Works](#-how-this-demo-works)
4. [Quick Start](#-quick-start)
5. [Command Line Tools](#-command-line-tools)
6. [Code Walkthrough](#-code-walkthrough)
7. [Key Concepts](#-key-concepts)

---

## 🤔 What is a Service?

A **Service** is a **Request/Response** communication pattern in ROS 2.

```
┌──────────────────┐                      ┌──────────────────┐
│                  │     REQUEST          │                  │
│     CLIENT       │ ──────────────────▶  │     SERVER       │
│                  │   "Add 5 + 3"        │                  │
│  (Asks question) │                      │   (Does work)    │
│                  │     RESPONSE         │                  │
│                  │ ◀──────────────────  │                  │
│                  │   "Result: 8"        │                  │
└──────────────────┘                      └──────────────────┘
```

### Real-World Analogy: Phone Call ☎️

| Service | Phone Call |
|---------|------------|
| Client sends request | You dial a number |
| Server receives request | Friend picks up |
| Server processes | Friend thinks of answer |
| Server sends response | Friend tells you answer |
| Client receives response | You hear the answer |
| Done! | Call ends |

---

## 🔄 Topics vs Services

| Aspect | Topics | Services |
|--------|--------|----------|
| **Pattern** | Publish/Subscribe | Request/Response |
| **Direction** | One-way | Two-way |
| **Timing** | Continuous streaming | One-time call |
| **Blocking** | Non-blocking | Can be blocking |
| **Use Case** | Sensor data, status | Commands, queries |
| **Analogy** | Radio broadcast 📻 | Phone call ☎️ |

### When to Use What?

```
┌─────────────────────────────────────────────────────────────────┐
│  USE TOPICS WHEN:                                               │
│  • Data is continuous (sensor readings)                         │
│  • Multiple nodes need the same data                            │
│  • Publisher doesn't care who receives                          │
│  • Examples: Camera images, robot position, sensor data         │
├─────────────────────────────────────────────────────────────────┤
│  USE SERVICES WHEN:                                              │
│  • You need a response                                           │
│  • It's a one-time request                                       │
│  • Action is quick (< few seconds)                               │
│  • Examples: Get parameter, toggle LED, calculate something      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🎯 How This Demo Works

### The Service Definition

We use `example_interfaces/srv/AddTwoInts`:

```
# Request (what client sends)
int64 a
int64 b
---
# Response (what server returns)
int64 sum
```

The `---` separates request from response.

### The Flow

```
Step 1: Server starts and waits
        ┌────────────────────┐
        │  CALCULATOR SERVER │
        │                    │
        │  "I'm ready!       │
        │   Waiting for      │
        │   requests..."     │
        └────────────────────┘

Step 2: Client sends request
        ┌────────────────────┐         ┌────────────────────┐
        │  CALCULATOR CLIENT │─────────│  CALCULATOR SERVER │
        │                    │  a=5    │                    │
        │  "Please add       │  b=3    │  "Request          │
        │   5 + 3"           │────────▶│   received!"       │
        └────────────────────┘         └────────────────────┘

Step 3: Server processes & responds
        ┌────────────────────┐         ┌────────────────────┐
        │  CALCULATOR CLIENT │◀────────│  CALCULATOR SERVER │
        │                    │  sum=8  │                    │
        │  "Got it!          │◀────────│  "5 + 3 = 8"       │
        │   Result is 8"     │         │   Sending...       │
        └────────────────────┘         └────────────────────┘

Step 4: Done! Client has the answer.
```

---

## 🚀 Quick Start

### Build the Package

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select service_demo
source install/setup.bash
```

### Run the Demo

**Terminal 1 - Start the Server:**
```bash
ros2 run service_demo server
```

**Terminal 2 - Send Requests with Client:**
```bash
# Add 5 + 3
ros2 run service_demo client 5 3

# Add 100 + 200
ros2 run service_demo client 100 200

# Add negative numbers
ros2 run service_demo client -10 25
```

### Expected Output

**Server Terminal:**
```
[INFO] 🧮 CALCULATOR SERVICE SERVER READY
[INFO] 📡 Service: /add_two_ints
[INFO] ⏳ Waiting for requests...
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] 📥 Request #1 received!
[INFO]    Input:  5 + 3
[INFO]    Output: 8
[INFO] 📤 Response sent!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**Client Terminal:**
```
[INFO] 🔍 Looking for calculator service...
[INFO] ✅ Service found! Ready to send requests.
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] 📤 Sending SYNC request: 5 + 3
[INFO] 📥 Response received: 8
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] ✅ RESULT: 5 + 3 = 8
```

---

## 🛠️ Command Line Tools

### List All Services

```bash
ros2 service list
```

Output:
```
/add_two_ints
/calculator_server/describe_parameters
...
```

### Get Service Type

```bash
ros2 service type /add_two_ints
```

Output:
```
example_interfaces/srv/AddTwoInts
```

### View Service Definition

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

Output:
```
int64 a
int64 b
---
int64 sum
```

### Call Service from Command Line (No Client Needed!)

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

Output:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=10, b=20)

response:
example_interfaces.srv.AddTwoInts_Response(sum=30)
```

### Get Service Info

```bash
ros2 service info /add_two_ints
```

---

## 📖 Code Walkthrough

### Server Code Structure

```python
# 1. Import the service type
from example_interfaces.srv import AddTwoInts

# 2. Create the service in __init__
self.service = self.create_service(
    AddTwoInts,           # Service type
    'add_two_ints',       # Service name
    self.add_callback     # Callback function
)

# 3. Define the callback
def add_callback(self, request, response):
    response.sum = request.a + request.b
    return response
```

### Client Code Structure

```python
# 1. Import the service type
from example_interfaces.srv import AddTwoInts

# 2. Create the client
self.client = self.create_client(AddTwoInts, 'add_two_ints')

# 3. Wait for service to be available
self.client.wait_for_service()

# 4. Create and send request
request = AddTwoInts.Request()
request.a = 5
request.b = 3
future = self.client.call_async(request)

# 5. Wait for response
rclpy.spin_until_future_complete(self, future)
result = future.result()
print(f"Sum = {result.sum}")
```

---

## 🧠 Key Concepts

### 1. Service Types

Services have a **type** that defines request and response:

```
example_interfaces/srv/AddTwoInts
       │              │      │
       │              │      └── Service name
       │              └── "srv" = service (vs "msg" for topics)
       └── Package name
```

### 2. Synchronous vs Asynchronous Calls

| Sync Call | Async Call |
|-----------|------------|
| `spin_until_future_complete()` | `call_async()` + callback |
| Blocks until response | Returns immediately |
| Simple to use | Better for real apps |
| Node stops working while waiting | Node keeps working |

### 3. Service Availability

Always check if service exists before calling:

```python
# Wait max 1 second for service
if not self.client.wait_for_service(timeout_sec=1.0):
    print("Service not available!")
```

### 4. Multiple Clients, One Server

```
┌─────────┐
│ Client 1│───┐
└─────────┘   │     ┌──────────┐
              ├────▶│  SERVER  │  (One server handles all)
┌─────────┐   │     └──────────┘
│ Client 2│───┘
└─────────┘
```

---

## 📊 Quick Reference

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 SERVICE QUICK REFERENCE                │
├─────────────────────────────────────────────────────────────────┤
│  PATTERN:      Request ──▶ Server ──▶ Response                  │
│                                                                  │
│  CREATE SERVER:                                                  │
│    self.create_service(ServiceType, 'name', callback)           │
│                                                                  │
│  CREATE CLIENT:                                                  │
│    self.create_client(ServiceType, 'name')                      │
│                                                                  │
│  CALL SERVICE:                                                   │
│    future = client.call_async(request)                          │
│                                                                  │
│  COMMAND LINE:                                                   │
│    ros2 service list                                             │
│    ros2 service call /name type "{field: value}"                │
│    ros2 service type /name                                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Package Structure

```
service_demo/
├── service_demo/
│   ├── __init__.py
│   ├── server.py      # Service server (adds numbers)
│   └── client.py      # Service client (sends requests)
├── resource/
│   └── service_demo
├── package.xml
├── setup.cfg
├── setup.py
└── README.md          # This file
```

---

## 🎯 Next Steps

After understanding Services, learn about **Actions**:
- Actions are for **long-running tasks**
- They provide **feedback** during execution
- They can be **cancelled**

Example: "Navigate to point" → Progress: 10%, 50%, 90% → Done!

---

*Happy Learning! 🤖*
