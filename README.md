## Описание

Данный код реализует алгоритм Дейкстры для нахождения кратчайших путей между вершинами во взвешенном графе. Вот более подробное объяснение его работы:

1. **Инициализация переменных**: 
   - Создаются словари `dist` и `preds`. Словарь `dist` используется для хранения кратчайших расстояний от начальной вершины до каждой другой вершины, изначально устанавливая все расстояния как бесконечность (кроме расстояния до начальной вершины, которое устанавливается равным 0). Словарь `preds` используется для хранения предшественников на кратчайших путях до каждой вершины, изначально устанавливая всех предшественников как `None`.
   - Создается список `unvisited`, содержащий все вершины графа.

2. **Основной цикл алгоритма**:
   - Пока есть непосещенные вершины в списке `unvisited`, выбирается вершина с наименьшим текущим расстоянием от начальной вершины. Это обеспечивает постепенное обновление кратчайших расстояний для всех вершин графа.
   - Для выбранной вершины обновляются расстояния до всех ее соседей, если новое расстояние меньше текущего.
   - Выбранная вершина помечается как посещенная путем удаления ее из списка `unvisited`.

3. **Функция `dijkstra(graph, start_node, end_node=None)`**:
   - Принимает в качестве параметров граф `graph`, начальную вершину `start_node` и, при необходимости, конечную вершину `end_node`. Если `end_node` не указана, то алгоритм вычислит кратчайшие расстояния от `start_node` до всех остальных вершин.
   - Возвращает словари `dist` и `preds`, содержащие кратчайшие расстояния от начальной вершины и предшественников на кратчайших путях для каждой вершины соответственно.

4. **Функция `get_path(preds, start_node, end_node)`**:
   - Используется для восстановления кратчайшего пути от начальной вершины `start_node` до конечной вершины `end_node` на основе словаря предшественников `preds`.
   - Возвращает список вершин, составляющих кратчайший путь от начальной вершины до конечной.

5. **Вывод результатов**:
   - По завершении работы алгоритма выводятся кратчайшие расстояния от начальной вершины до всех остальных вершин графа, а также, если указана конечная вершина, выводится кратчайший путь от начальной вершины до конечной.

## Результат работы программы

**Исходные данные**

G = nx.Graph()  
G.add_edge('A', 'B', weight=1)  
G.add_edge('A', 'C', weight=4)  
G.add_edge('B', 'C', weight=2)  
G.add_edge('B', 'D', weight=5)  
G.add_edge('C', 'D', weight=1)  
  
start_node = 'A'  
end_node = 'D'  
  
**Результат**  
Кратчайшие расстояния от вершины 'A' до всех остальных вершин:  
До вершины A: Расстояние = 0  
До вершины B: Расстояние = 1  
До вершины C: Расстояние = 3  
До вершины D: Расстояние = 4  
  
Кратчайший путь от вершины 'A' до вершины 'D': A -> B -> C -> D  
