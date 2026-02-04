#!/usr/bin/env python3
"""
CALCULATOR SERVICE SERVER
=========================
This node provides a SERVICE that adds two integers.

Service: /add_two_ints
Type: example_interfaces/srv/AddTwoInts
    Request:  int64 a, int64 b
    Response: int64 sum

How Services Work:
    1. Server starts and WAITS for requests
    2. Client sends a REQUEST with data (a, b)
    3. Server processes and sends RESPONSE (sum)
    4. Client receives the response

This is like a FUNCTION CALL over the network!
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class CalculatorServer(Node):
    """A simple service server that adds two numbers."""

    def __init__(self):
        super().__init__('calculator_server')
        
        # =====================================================
        # CREATE THE SERVICE
        # =====================================================
        # create_service(service_type, service_name, callback)
        self.service = self.create_service(
            AddTwoInts,           # Service type (from example_interfaces)
            'add_two_ints',       # Service name
            self.add_callback     # Function to call when request arrives
        )
        
        self.request_count = 0
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🧮 CALCULATOR SERVICE SERVER READY')
        self.get_logger().info('=' * 50)
        self.get_logger().info('📡 Service: /add_two_ints')
        self.get_logger().info('📋 Type: example_interfaces/srv/AddTwoInts')
        self.get_logger().info('⏳ Waiting for requests...')
        self.get_logger().info('=' * 50)

    def add_callback(self, request, response):
        """
        Called when a client sends a request.
        
        Args:
            request: Contains 'a' and 'b' (the two numbers)
            response: We fill in 'sum' (the result)
        
        Returns:
            response: The completed response
        """
        self.request_count += 1
        
        # Perform the calculation
        result = request.a + request.b
        
        # Fill in the response
        response.sum = result
        
        # Log the operation
        self.get_logger().info('━' * 50)
        self.get_logger().info(f'📥 Request #{self.request_count} received!')
        self.get_logger().info(f'   Input:  {request.a} + {request.b}')
        self.get_logger().info(f'   Output: {result}')
        self.get_logger().info('📤 Response sent!')
        self.get_logger().info('━' * 50)
        
        # Return the response
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CalculatorServer()
    
    try:
        # spin() keeps the node running and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'👋 Shutting down. Processed {node.request_count} requests.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
