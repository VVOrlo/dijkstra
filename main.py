import networkx as nx
from math import inf

def dijkstra(graph, start_node):
    # Создаем словарь для хранения расстояний
    distances = {}
    for node in graph.nodes():
        distances[node] = inf
    distances[start_node] = 0

    # Создаем словарь для хранения предшественников
    predecessors = {}
    for node in graph.nodes():
        predecessors[node] = None

    # Создаем список непосещенных вершин
    unvisited_nodes = list(graph.nodes())

    while unvisited_nodes:
        # Находим вершину с наименьшим расстоянием
        min_distance = inf
        current_node = None
        for node in unvisited_nodes:
            if distances[node] < min_distance:
                min_distance = distances[node]
                current_node = node

        # Удаляем текущую вершину из непосещенных
        unvisited_nodes.remove(current_node)

        # Если расстояние до текущей вершины равно бесконечности, значит, достигли несвязанных вершин
        if distances[current_node] == inf:
            break

        # Проходим по соседям текущей вершины
        for neighbor in graph.neighbors(current_node):
            # Вычисляем новое расстояние от начальной вершины до соседа
            distance = distances[current_node] + graph[current_node][neighbor]['weight']

            # Если новое расстояние меньше текущего, обновляем расстояние и предшественника
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                predecessors[neighbor] = current_node

    return distances, predecessors

# Пример использования
if __name__ == "__main__":
    # Создаем граф
    G = nx.Graph()
    G.add_edge('A', 'B', weight=1)
    G.add_edge('A', 'C', weight=4)
    G.add_edge('B', 'C', weight=2)
    G.add_edge('B', 'D', weight=5)
    G.add_edge('C', 'D', weight=1)

    # Запускаем алгоритм Дейкстры
    distances, predecessors = dijkstra(G, 'A')

    # Выводим кратчайшие расстояния
    print("Кратчайшие расстояния от вершины 'A':", distances)