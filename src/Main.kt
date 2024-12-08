import java.util.*

fun main() {
    val graph = Graph()
    val book = Vertex("book")
    val vinyl = Vertex("vinyl")
    val poster = Vertex("poster")
    val guitar = Vertex("guitar")
    val drum = Vertex("drum")
    val piano = Vertex("piano")

    graph.addVertex(book)
    graph.addVertex(vinyl)
    graph.addVertex(poster)
    graph.addVertex(guitar)
    graph.addVertex(drum)
    graph.addVertex(piano)

    graph.addEdge(from = book, to = vinyl, weight = 5)
    graph.addEdge(from = book, to = poster, weight = 2)
    graph.addEdge(from = vinyl, to = guitar, weight = 15)
    graph.addEdge(from = vinyl, to = drum, weight = 20)
    graph.addEdge(from = poster, to = guitar, weight = 30)
    graph.addEdge(from = poster, to = drum, weight = 35)
    graph.addEdge(from = guitar, to = piano, weight = 20)
    graph.addEdge(from = drum, to = piano, weight = 10)

    val result = dijkstra(graph = graph, start = book, end = piano)
    if (result == null) {
        println("Путь не найден")
        return
    }
    println("Цена от ${book.name} до ${piano.name}: ${result.cost}")
    println("Путь от ${book.name} до ${piano.name}: ${result.path.joinToString(" -> ") { it.name }}")
}

/**
 * Узел графа
 *
 * @property name Название узла
 */
data class Vertex(
    val name: String,
)

/**
 * Ребро графа
 *
 * @property to Узел, к которому ведет ребро
 * @property cost Цена ребра
 */
data class Edge(
    val to: Vertex,
    val cost: Int,
)

/**
 * Результат работы алгоритма Дейкстры
 *
 * @property cost Цена кратчайшего пути от начальной до финальной точки
 * @property path Кратчайший путь представленный в виде списка
 */
data class DijkstraResult(
    val cost: Int,
    val path: List<Vertex>,
)

/**
 * Класс для хранения и редактирования графа.
 * Внутри себя хранит мап со списком узлов и ребер, которыми связаны узлы
 */
class Graph {
    private val graphMap: MutableMap<Vertex, MutableList<Edge>> = mutableMapOf()

    fun addVertex(vertex: Vertex) {
        graphMap.putIfAbsent(vertex, mutableListOf())
    }

    fun addEdge(from: Vertex, to: Vertex, weight: Int) {
        graphMap[from]?.add(Edge(to, weight))
    }

    fun getEdges(vertex: Vertex) = graphMap[vertex] ?: listOf()

    fun getGraphMap(): Map<Vertex, MutableList<Edge>> {
        return graphMap
    }
}

fun dijkstra(graph: Graph, start: Vertex, end: Vertex): DijkstraResult? {
    // Хранилище цен перехода от начального узла
    val costs = mutableMapOf<Vertex, Int>()

    // Хранилище связей [родитель - ребенок] для построения кратчайшего пути
    val parents = mutableMapOf<Vertex, Vertex>()

    // Хранилище посещенных узлов графа
    val visitedVertexes = mutableSetOf<Vertex>()

    // Очередь с приоритетом чтобы не надо было искать каждый раз узел с минимальной ценой в неосортированной коллекции
    val priorityQueue = PriorityQueue<Pair<Int, Vertex>>(compareBy { it.first })

    // Инициализируем расстояния
    graph.getGraphMap().keys.forEach { vertex ->
        costs[vertex] = Int.MAX_VALUE
    }

    // Расстояние от источника до самого себя равно 0
    costs[start] = 0

    // Добавляем источник в очередь
    priorityQueue.add((costs[start] ?: Int.MAX_VALUE) to start)

    while (priorityQueue.isNotEmpty()) {
        // Извлекаем вершину с наименьшей ценой из очереди
        val (currentVertexCost, currentVertex) = priorityQueue.poll()

        // Если текущая вершина была обработана, пропускаем ее
        if (visitedVertexes.contains(currentVertex)) {
            continue
        }
        // Добавляем текущую вершину в множество посещенных
        visitedVertexes.add(currentVertex)

        // Цена текущей вершины выше уже существующем, то пропускаем этот шаг (Отбрасываем вариант)
        if (currentVertexCost > (costs[currentVertex] ?: Int.MAX_VALUE)) {
            continue
        }

        // Обходим соседние вершины, к которым ведут ребра
        for (currentVertextEdge in graph.getEdges(currentVertex)) {
            // Рассчитываем новую цену
            val newCost = currentVertexCost + currentVertextEdge.cost

            // Если новая цена меньше известной
            if (newCost < (costs[currentVertextEdge.to] ?: Int.MAX_VALUE)) {
                // Обновляем цену узла, к которому ведет ребро
                costs[currentVertextEdge.to] = newCost
                // Добавляем связь [родитель - ребенок] чтобы можно было вывести кратчайший путь
                parents[currentVertextEdge.to] = currentVertex
                // Добавляем в очередь узел, к которому ведет ребро
                priorityQueue.add(newCost to currentVertextEdge.to)
            }
        }
    }

    val cost = costs[end] ?: return null

    val path = mutableListOf<Vertex>()

    var current = end
    path.add(0, end)
    while (current != start) {
        val next = parents[current] ?: return null
        path.add(0, next)
        current = next
    }

    return DijkstraResult(
        cost = cost,
        path = path,
    )
}
