import operator


class Heap:
    """"
    Attributes:
        heap: List representation of the heap
        compare(p, c): comparator function, returns true if the relation between p and c is parent-child
    """

    def __init__(self, heap=None, heapX=None, heapY=None, compare=operator.lt):
        self.heap = [] if heap is None else heap
        self.heapX = [] if heapX is None else heapX
        self.heapY = [] if heapY is None else heapY
        self.compare = compare

    def __repr__(self):
        return 'Heap({!r}, {!r}, {!r}, {!r})'.format(self.heap, self.heapX, self.heapY, self.compare)

    def _inv_heapify(self, child_index):
        """
        Do heapifying starting from bottom till it reaches the root.
        """
        heap, heapX, heapY, compare = self.heap, self.heapX, self.heapY, self.compare
        child = child_index
        while child > 0:
            parent = (child - 1) // 2
            if compare(heap[parent], heap[child]):
                return
            heap[parent], heap[child] = heap[child], heap[parent]
            heapX[parent], heapX[child] = heapX[child], heapX[parent]
            heapY[parent], heapY[child] = heapY[child], heapY[parent]
            child = parent

    def _heapify(self, parent_index):
        """
        Do heepifying starting from the root.
        """
        heap, heapX, heapY, compare = self.heap, self.heapX, self.heapY, self.compare
        length = len(heap)
        if length == 1:
            return
        parent = parent_index
        while 2 * parent + 1 < length:
            child = 2 * parent + 1
            if child + 1 < length and compare(heap[child + 1], heap[child]):
                child += 1
            if compare(heap[parent], heap[child]):
                return
            heap[parent], heap[child] = heap[child], heap[parent]
            heapX[parent], heapX[child] = heapX[child], heapX[parent]
            heapY[parent], heapY[child] = heapY[child], heapY[parent]
            parent = child

    def del_min(self):
        heap = self.heap
        heapX = self.heapX
        heapY = self.heapY
        last_element = heap.pop()
        last_elementX = heapX.pop()
        last_elementY = heapY.pop()
        if not heap:
            return last_element
        heap[0] = last_element
        heapX[0] = last_elementX
        heapY[0] = last_elementY
        self._heapify(0)
        # return item

    def min(self):
        if not self.heap:
            return None
        return self.heap[0]

    def minX(self):
        if not self.heapX:
            return None
        return self.heapX[0]

    def minY(self):
        if not self.heapY:
            return None
        return self.heapY[0]

    def add(self, v, x, y):
        self.heap.append(v)
        self.heapX.append(x)
        self.heapY.append(y)
        self._inv_heapify(len(self.heap) - 1)