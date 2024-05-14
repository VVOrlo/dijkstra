import networkx as nx
from math import inf

def dijkstra(graph, start_node):
    # Создаем словарь для хранения расстояний
    dist = {n: inf for n in graph.nodes()}
    dist[start_node] = 0

    # Создаем словарь для хранения предшественников
    preds = {n: None for n in graph.nodes()}

    # Создаем список непосещенных вершин
    unvisited = list(graph.nodes())

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

    return dist, preds

def get_path(preds, start_node, end_node):
    path = []
    current_node = end_node
    while current_node is not None:
        path.insert(0, current_node)
        current_node = preds[current_node]
    if path[0] == start_node:
        return path
    else:
        return None

# Пример использования
if __name__ == "__main__":
    # Создаем граф
    G = nx.Graph()
    G.add_edge('A', 'B', weight=1)
    G.add_edge('A', 'C', weight=4)
    G.add_edge('B', 'C', weight=2)
    G.add_edge('B', 'D', weight=5)
    G.add_edge('C', 'D', weight=1)

    start_node = 'B'  # Можно изменить на любую стартовую вершину

    # Запускаем алгоритм Дейкстры
    distances, predecessors = dijkstra(G, start_node)

    # Выводим кратчайшие расстояния
    print(f"Кратчайшие расстояния от вершины '{start_node}':")
    for node, distance in distances.items():
        print(f"До вершины {node} : Расстояние = {distance}")

    # Выводим кратчайшие пути
    print(f"\nКратчайшие пути от вершины '{start_node}':")
    for node in G.nodes():
        if node != start_node:
            path = get_path(predecessors, start_node, node)
            if path:
                print(f"Путь до вершины {node}: {' -> '.join(path)}")
            else:
                print(f"Путь до вершины {node} не найден.")
