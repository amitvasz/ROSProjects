import heapq


class Elem(object):
    def __init__(self, x, y, reachable):
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0


class AStar(object):
    def __init__(self, grid, start, end):
        self.open_cells = []
        heapq.heapify(self.open_cells)
        self.closed = set()
        self.cells = []
        self.grid_height = len(grid)
        self.grid_width = len(grid[0])
        for x in xrange(len(grid[0])):
            for y in xrange(len(grid)):
                self.cells.append(Elem(x, y, 1-grid[y][x]))
        self.start = self.get_elem(*start)
        self.end = self.get_elem(*end) 

    def get_elem(self, x, y):
        return self.cells[x * self.grid_height + y]

    def solve(self):
        heapq.heappush(self.open_cells, (self.start.f_cost, self.start))
        while len(self.open_cells):
            f, cell = heapq.heappop(self.open_cells)
            self.closed.add(cell)
            if cell is self.end:
                cell = self.end
                path = [(cell.x, cell.y)]
                while cell.parent is not self.start:
                    cell = cell.parent
                    path.append((cell.x, cell.y))

                path.append((self.start.x, self.start.y))
                path.reverse()
                return path
            adj_cells = []
            if cell.x < self.grid_width-1:
                adj_cells.append(self.get_elem(cell.x+1, cell.y))
            if cell.y > 0:
                adj_cells.append(self.get_elem(cell.x, cell.y-1))
            if cell.x > 0:
                adj_cells.append(self.get_elem(cell.x-1, cell.y))
            if cell.y < self.grid_height-1:
                adj_cells.append(self.get_elem(cell.x, cell.y+1))
            for adj_cell in adj_cells:
                if adj_cell.reachable == 1 and adj_cell not in self.closed:
                    if (adj_cell.f_cost, adj_cell) in self.open_cells:
                        if adj_cell.g_cost > cell.g_cost + 10:
                            adj_cell.g_cost = cell.g_cost + 10
                            adj_cell.h_cost = (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))
                            adj_cell.parent = cell
                            adj_cell.f_cost = adj_cell.h_cost + adj_cell.g_cost
                    else:
                        adj_cell.g_cost = cell.g_cost + 10
                        adj_cell.h_cost = (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))
                        adj_cell.parent = cell
                        adj_cell.f_cost = adj_cell.h_cost + adj_cell.g_cost
                        heapq.heappush(self.open_cells, (adj_cell.f_cost, adj_cell))

if __name__ == "__main__":
    map_arr =  [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
                [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
                [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

    astar = AStar(map_arr,(1,11),(13,1))
    path = astar.solve()
    for i in xrange(len(map_arr)):
        for j in xrange(len(map_arr[0])):
            if (j,i) in path:
                print '*',
            else:
                print map_arr[i][j],
        print