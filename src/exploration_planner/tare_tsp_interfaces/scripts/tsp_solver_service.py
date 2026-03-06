#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tare_tsp_interfaces.srv import SolveTSP
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


class TSPSolverService(Node):
    def __init__(self):
        super().__init__('tsp_solver_service')
        self.srv = self.create_service(
            SolveTSP, 
            'solve_tsp', 
            self.solve_tsp_callback
        )
        self.get_logger().info('TSP Solver Service ready')

    def solve_tsp_callback(self, request, response):
        try:
            # Reconstruct distance matrix
            n = request.matrix_size
            distance_matrix = [
                request.distance_matrix[i*n:(i+1)*n] 
                for i in range(n)
            ]
            
            # Solve TSP
            node_seq, path_len = self._solve_tsp(
                distance_matrix, 
                request.depot
            )
            
            # Handle dummy node if needed
            if request.has_dummy and node_seq:
                node_seq = self._process_dummy(node_seq, n)
            
            response.node_sequence = node_seq
            response.path_length = path_len
            response.success = True
            response.message = f"Solved for {len(node_seq)} nodes"
            
        except Exception as e:
            self.get_logger().error(f'TSP failed: {str(e)}')
            response.success = False
            response.message = str(e)
            response.node_sequence = []
            response.path_length = 0.0
        
        return response
    
    def _solve_tsp(self, distance_matrix, depot):
        num_nodes = len(distance_matrix)
        manager = pywrapcp.RoutingIndexManager(num_nodes, 1, depot)
        routing = pywrapcp.RoutingModel(manager)
        
        def distance_callback(from_idx, to_idx):
            from_node = manager.IndexToNode(from_idx)
            to_node = manager.IndexToNode(to_idx)
            return distance_matrix[from_node][to_node]
        
        transit_idx = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)
        
        search_params = pywrapcp.DefaultRoutingSearchParameters()
        search_params.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        )
        
        solution = routing.SolveWithParameters(search_params)
        
        if solution:
            node_sequence = []
            index = routing.Start(0)
            while not routing.IsEnd(index):
                node_sequence.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            path_length = solution.ObjectiveValue() / 10.0
            return node_sequence, path_length
        
        return [], 0.0
    
    def _process_dummy(self, node_seq, matrix_size):
        dummy_idx = matrix_size - 1
        if len(node_seq) > 1 and node_seq[1] == dummy_idx:
            node_seq.pop(1)
            if node_seq:
                node_seq.append(node_seq[0])
                node_seq.pop(0)
                node_seq.reverse()
        elif node_seq and node_seq[-1] == dummy_idx:
            node_seq.pop()
        return node_seq


def main():
    rclpy.init()
    service = TSPSolverService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


