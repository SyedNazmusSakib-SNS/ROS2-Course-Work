#!/usr/bin/env python3
"""
CALCULATOR SERVICE CLIENT
=========================
This node sends requests to the calculator service.

It demonstrates TWO ways to call a service:
    1. SYNCHRONOUS: Wait for response (blocking)
    2. ASYNCHRONOUS: Continue working while waiting (non-blocking)

Usage:
    ros2 run service_demo client 5 3
    ros2 run service_demo client 100 200
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorClient(Node):
    """A simple service client that requests addition."""

    def __init__(self):
        super().__init__('calculator_client')
        
        # =====================================================
        # CREATE THE SERVICE CLIENT
        # =====================================================
        self.client = self.create_client(
            AddTwoInts,       # Service type
            'add_two_ints'    # Service name (must match server!)
        )
        
        # =====================================================
        # WAIT FOR SERVICE TO BE AVAILABLE
        # =====================================================
        self.get_logger().info('🔍 Looking for calculator service...')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('⏳ Service not available, waiting...')
        
        self.get_logger().info('✅ Service found! Ready to send requests.')

    def send_request_sync(self, a, b):
        """
        SYNCHRONOUS call - waits for response.
        Simple but blocks the node.
        """
        self.get_logger().info('━' * 50)
        self.get_logger().info(f'📤 Sending SYNC request: {a} + {b}')
        
        # Create the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Call the service and WAIT for response
        future = self.client.call_async(request)
        
        # Spin until we get a response
        rclpy.spin_until_future_complete(self, future)
        
        # Get the result
        result = future.result()
        
        self.get_logger().info(f'📥 Response received: {result.sum}')
        self.get_logger().info('━' * 50)
        
        return result.sum

    def send_request_async(self, a, b):
        """
        ASYNCHRONOUS call - doesn't block.
        Better for real applications where node needs to keep running.
        """
        self.get_logger().info('━' * 50)
        self.get_logger().info(f'📤 Sending ASYNC request: {a} + {b}')
        
        # Create the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Call the service (non-blocking)
        future = self.client.call_async(request)
        
        # Add a callback for when the response arrives
        future.add_done_callback(self.response_callback)
        
        self.get_logger().info('⏳ Request sent, waiting for response...')
        
        return future

    def response_callback(self, future):
        """Called when async response arrives."""
        result = future.result()
        self.get_logger().info(f'📥 Async response received: {result.sum}')
        self.get_logger().info('━' * 50)


def main(args=None):
    rclpy.init(args=args)
    
    # Get numbers from command line arguments
    if len(sys.argv) >= 3:
        try:
            a = int(sys.argv[1])
            b = int(sys.argv[2])
        except ValueError:
            print('Usage: ros2 run service_demo client <int_a> <int_b>')
            print('Example: ros2 run service_demo client 5 3')
            return
    else:
        # Default values if no arguments given
        a = 5
        b = 3
    
    # Create client and send request
    client = CalculatorClient()
    
    try:
        # Use synchronous call for simplicity
        result = client.send_request_sync(a, b)
        client.get_logger().info(f'✅ RESULT: {a} + {b} = {result}')
        
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
