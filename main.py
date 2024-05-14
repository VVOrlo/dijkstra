import networkx as nx
from math import inf

def dijkstra(graph, start_node, end_node=None):
    # Создаем словарь для хранения расстояний, изначально все расстояния бесконечны
    dist = {n: inf for n in graph.nodes()}
    # Расстояние от начальной вершины до нее самой равно 0
    dist[start_node] = 0

    # Создаем словарь для хранения предшественников, изначально все предшественники None
    preds = {n: None for n in graph.nodes()}

    # Создаем список непосещенных вершин
    unvisited = list(graph.nodes())

    # Пока есть непосещенные вершины
    while unvisited:
        # Находим вершину с наименьшим расстоянием
        min_dist = inf
        curr_node = None
        for n in unvisited:
            if dist[n] < min_dist:
                min_dist = dist[n]
                curr_node = n

        # Удаляем текущую вершину из непосещенных
        unvisited.remove(curr_node)

        # Если расстояние до текущей вершины равно бесконечности, значит, достигли несвязанных вершин
        if dist[curr_node] == inf:
            break

        # Проходим по соседям текущей вершины
        for neighbor in graph.neighbors(curr_node):
            # Вычисляем новое расстояние от начальной вершины до соседа
            d = dist[curr_node] + graph[curr_node][neighbor]['weight']
            # Если новое расстояние меньше текущего, обновляем расстояние и предшественника
            if d < dist[neighbor]:
                dist[neighbor] = d
                preds[neighbor] = curr_node

    # Если конечная вершина указана, возвращаем кратчайший путь до нее
    if end_node:
        path = []
        current_node = end_node
        while current_node is not None:
            path.insert(0, current_node)
            current_node = preds[current_node]
        return dist, path

    # В противном случае возвращаем кратчайшие расстояния до всех остальных вершин
    return dist, preds

# Пример использования
if __name__ == "__main__":
    # Создаем граф
    G = nx.Graph()
    G.add_edge('A', 'B', weight=1)
    G.add_edge('A', 'C', weight=4)
    G.add_edge('B', 'C', weight=2)
    G.add_edge('B', 'D', weight=5)
    G.add_edge('C', 'D', weight=1)

    # Начальная и конечная вершины
    start_node = 'A'
    end_node = 'D'

    # Запускаем алгоритм Дейкстры
    distances, path = dijkstra(G, start_node, end_node)

    # Выводим кратчайшие расстояния и путь до указанной конечной вершины
    print(f"Кратчайшие расстояния от вершины '{start_node}' до всех остальных вершин:")
    for node, distance in distances.items():
        print(f"До вершины {node} : Расстояние = {distance}")
    print(f"Кратчайший путь от вершины '{start_node}' до вершины '{end_node}': {' -> '.join(path)}")
