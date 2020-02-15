import sys
import numpy as np
import pygame
import Heap
import math
import time

offsetx = [0, 1, 0, -1]
offsety = [1, 0, -1, 0]


def RandomColor(num):
    black = (0, 0, 0)
    white = (255, 255, 255)
    maroon = (128, 0, 0)
    red = (255, 0, 0)
    steel_blue = (70, 130, 180)
    gold = (255, 215, 0)
    dark_orange = (255, 140, 0)
    dark_golden = (184, 134, 11)
    peru = (205, 133, 63)
    purple = (128, 0, 128)
    medium_orchid = (186, 85, 211)
    navy = (0, 0, 128)
    pink = (255, 20, 147)
    orange = (255, 69, 0)
    blue_sky = (135, 206, 250)
    olive = (128, 128, 0)
    tomato = (255, 99, 71)
    salmon = (250, 128, 114)
    indian_red = (205, 92, 92)
    crimson = (220, 20, 60)
    pale_golden = (238, 232, 170)
    spring_green = (0, 255, 127)
    light_sea = (32, 178, 170)
    dark_violet = (148, 0, 211)
    pale_violet = (219, 112, 147)
    chocolate = (210, 105, 30)
    rosy_brown = (188, 143, 143)
    if (num == 0):
        return white
    if (num == 1):
        return steel_blue
    if (num == 2):
        return maroon
    if (num == 3):
        color_of_pick_point = [dark_golden, gold, dark_orange, tomato, salmon, indian_red, crimson, pale_golden, spring_green, black]
        return color_of_pick_point
    if (num == 4):
        color_of_polygon = [red, peru, navy, orange, pink, blue_sky, olive, light_sea, dark_violet, pale_violet, chocolate,
                rosy_brown]
        return color_of_polygon
    if (num == 5):
        return medium_orchid
    if (num == 6):
        return purple
class Point:
    def __init__(self, X, Y):
        self.x = int(X)
        self.y = int(Y)
    def setXP(self, x):
        self.x = x
    def setYP(self, y):
        self.y = y
    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def PrintPoint(self):
        print(self.x, self.y)
    def PSPoint(self):
        print('S: ', self.x, self.y)
    def PGPoint(self):
        print('G: ', self.x, self.y)
    def PPickupPoint(self):
        print('Pickup Point: ', self.x, self.y)

class Polygon:
    def __init__(self, amount):
        self.quantum = amount
        self.list = []
        self.Pol = []
    def pushPoint(self, Point):
        self.list.append(Point)
    def pushPoly(self, Polygon):
        self.Pol.append(Polygon)
    def PrintSerialPoly(self):
        for i in range(int(self.quantum)):
            print('Polygon thu ', i + 1, ': ')
            for j in range(len(self.Pol[i].list)):
                print(self.Pol[i].list[j].x, self.Pol[i].list[j].y)

class Map:
    def __init__(self, depth, width):
        self.depth= int(depth)
        self.width= int(width)
        self.Spoint = Point(0, 0)
        self.Gpoint = Point(0, 0)
        self.Ppoint = []
        self.lPol = []
        self.poly = Polygon(0)
        self.path = []
    def setSpoint(self, Point):
        self.Spoint.x = Point.x
        self.Spoint.y = Point.y
    def setGpoint(self, Point):
        self.Gpoint.x = Point.x
        self.Gpoint.y = Point.y
    def setPickpoint(self, Point):
        self.Ppoint.append(Point)
    def setPolyquantum(self, n):
        self.poly.quantum = n
    def setpushPoint(self,Point):
        self.poly.pushPoint(Point)
    def setlistPol(self,Polygon):
        self.poly.pushPoly(Polygon)
    def getPickPoint(self):
        return self.Ppoint

    def getPpoint(self, i):
        return self.Ppoint[i]

    def printPath(self, A, m, n):
        for i in range(1, m):
            for j in range(1, n):
                if A[i][j] == '':
                    print('.\t', end='')
                elif A[i][j] == '1':
                    A[i][j] = '+'
                else:
                    print(A[i][j], '\t', end='')
            print()
    def count_p(self, path):
        tmp = []
        i = 0
        while i < len(path):
            a = path[i]
            s = 0
            for j in range(len(path)):
                if a[0] == path[j][0] and a[1] == path[j][1]:
                    s = s + 1
            tmp.append((a[0], a[1], s))
            i+=1
        return tmp
    def Bresenham(self, A, B):
        path = []
        x0 = int(A.x)
        y0 = int(A.y)
        x1 = int(B.x)
        y1 = int(B.y)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                path.append((x,y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                path.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        path.append((x, y))
        return path
    def FindpathDFS(self):
        graph = Map(0, 0)
        graph = self
        search = Search(graph)
        x_start = self.Spoint.x
        y_start = self.Spoint.y
        Gx = self.Gpoint.x
        Gy = self.Gpoint.y
        npath = []
        pathi = []
        i = 0
        while i < 6:
            pathi.append(search.DFS(x_start, y_start, Gx, Gy))
            npath.append(search.distance(pathi[i]))
            i += 1
        min_path = search.mindistance(npath)
        index = min_path[0]
        ret = []
        for i in range(len(pathi[index])-1):
            ret.append((pathi[index][i][0], pathi[index][i][1]))
        for i in range(len(self.Ma)):
            for j in range(len(self.Ma[0])):
                if self.Ma[i][j] == '1':
                    self.Ma[i][j] = ''
        return ret
    def DrawP(self):
        self.Ma = np.zeros((self.depth, self.width), dtype='U')
        print('Depth (m):', self.depth, '\nWidth (n):', self.width)
        self.Ma[self.Spoint.x][self.Spoint.y] = 'S'
        self.Ma[self.Gpoint.x][self.Gpoint.y] = 'G'
        for i in range(len(self.Ppoint)):
            self.Ma[self.Ppoint[i].x][self.Ppoint[i].y] = 'P'
        lPol = []
        for i in range(int(self.poly.quantum)):
            j = 0
            ar = []
            while j < len(self.poly.Pol[i].list) - 1:
                li = self.Bresenham(self.poly.Pol[i].list[j], self.poly.Pol[i].list[j + 1])
                ar.append(li)
                j += 1
                if j == len(self.poly.Pol[i].list) - 1:
                    li = self.Bresenham(self.poly.Pol[i].list[0], self.poly.Pol[i].list[len(self.poly.Pol[i].list) - 1])
                    ar.append(li)
            self.lPol.append(ar)
        for i in range(len(self.lPol)):
            for j in range(len(self.lPol[i])):
                for k in range(len(self.lPol[i][j])):
                    for m in range(len(self.lPol[i][j][k])):
                        self.Ma[int(self.lPol[i][j][k][0])][int(self.lPol[i][j][k][1])] = '#'
    def Illustration(self, method):
        cost = 0
        if (self.Ppoint):
            tmp = Search(self)
            fPath = tmp.Ast()
            if (fPath):
                for i in range(len(fPath)):
                    x = fPath[i].getX()
                    y = fPath[i].getY()
                    check = 0
                    if (x == self.Spoint.x and y == self.Spoint.y) or (x == self.Gpoint.x and y == self.Gpoint.y):
                        i+=1
                    else:
                        for j in range(len(self.Ppoint)):
                            if x == self.Ppoint[j].getX() and y == self.Ppoint[j].getY():
                                check +=1
                        if check !=0:
                            i+=1
                        else:
                            self.Ma[x][y] = '+'
                    self.path.append((x, y))
            cost = tmp.distance(self.path)
            print('Cost:', cost)
            T = self.count_p(self.path)
            self.path = T
            self.printPath(self.Ma, self.depth, self.width)
        else:
            if (method == 'DFS'):
                res = self.FindpathDFS()
                for i in range(len(res)):
                    self.Ma[res[i][0]][res[i][1]] = '+'
                    self.path.append((res[i][0], res[i][1]))
                self.path = self.path[::-1]
                self.path.append((self.Gpoint.x, self.Gpoint.y))
                ret = Search(self)
                cost = ret.distance(self.path)
                print('Cost:', cost)
                self.printPath(self.Ma, self.depth, self.width)
            if (method == 'BFS'):
                res = Search(self)
                self.path = res.BFS(self.Spoint, self.Gpoint)
                self.path = self.path[::-1]
                self.path.append((self.Gpoint.x, self.Gpoint.y))
                self.path = self.path[::-1]
                cost = res.distance(self.path)
                self.path = self.path[::-1]
                for i in range(len(self.path)-1):
                    self.Ma[int(self.path[i][0])][int(self.path[i][1])] = '+'
                print('Cost:', cost)
                self.printPath(self.Ma, self.depth, self.width)
            if (method == 'A*'):
                tmp = Search(self)
                fPath = tmp.Ast()
                if (fPath):
                    for i in range(len(fPath)):
                        x = fPath[i].getX()
                        y = fPath[i].getY()
                        if (x == self.Spoint.x and y == self.Spoint.y) or (
                                x == self.Gpoint.x and y == self.Gpoint.y):
                            i += 1
                        else:
                            self.Ma[x][y] = '+'
                        self.path.append((x, y))
                cost = tmp.distance(self.path)
                print('Cost:', cost)
                self.printPath(self.Ma, self.depth, self.width)
    # Set width and depth for grid
        width = 30
        depth = 30
        # This sets the margin between each cell
        margin = 2
        # Create a 2 dimensional array. A two dimensional
        # array is simply a list of lists.
        grid = []
        color_grid = RandomColor(0)
        color_S = RandomColor(1)
        color_G = RandomColor(2)
        color_Ppoint = RandomColor(3)
        color_Poly = RandomColor(4)
        color_path = RandomColor(5)
        coincidence = RandomColor(6)
        for row in range(10):
            # Add an empty array that will hold each cell
            # in this row
            grid.append([])
            for column in range(10):
                grid[row].append(0)  # Append a cell
        pygame.init()
        screen = pygame.display.set_mode((900, 750))
        done = False
        pygame.display.set_caption('Robot Tim Duong')
        font = pygame.font.Font(pygame.font.get_default_font(), 12)
        textS = font.render('S', True, color_grid)
        textG = font.render('G', True, color_grid)
        textP = font.render('P', True, color_grid)
        TEXT = font.render('Cost: ', True, color_grid)
        cost_value = font.render(str(cost), True, color_grid)
        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            for column in range(1, int(self.depth)):
                for row in range(1, int(self.width)):
                    color = color_grid
                    pygame.draw.rect(screen, color,
                                     [(margin + width) * column + margin, (margin + depth) * row + margin, width,
                                      depth])
            pygame.draw.rect(screen, color_S, [(margin + width) * self.Spoint.x + margin,
                                                 (margin + depth) * (self.width - self.Spoint.y) + margin, width,
                                                 depth])
            screen.blit(TEXT, ((margin + width) * 0 + margin, (margin + depth) * 0.25 + margin))
            screen.blit(cost_value, ((margin + width) * 1.25 + margin, (margin + depth) * 0.25 + margin))
            Scenterx = ((margin + width) * (self.Spoint.x + 0.35) + margin)
            Scentery = ((margin + depth) * (self.width - self.Spoint.y + 0.25) + margin)
            screen.blit(textS, (Scenterx, Scentery))
            pygame.draw.rect(screen, color_G, [(margin + width) * self.Gpoint.x + margin,
                                             (margin + depth) * (self.width - self.Gpoint.y) + margin, width,
                                             depth])
            for i in range(len(self.Ppoint)):
                Pcenterx = (margin + width) * (self.Ppoint[i].x + 0.35) + margin
                Pcentery = (margin + depth) * (self.width - self.Ppoint[i].y + 0.25) + margin
                pygame.draw.rect(screen, color_Ppoint[i], [(margin + width) * self.Ppoint[i].x + margin,
                                                  (margin + depth) * (self.width - self.Ppoint[i].y) + margin, width,
                                                  depth])
                screen.blit(textP, (Pcenterx, Pcentery))
            Gcenterx = ((margin + width) * (self.Gpoint.x + 0.35) + margin)
            Gcentery = ((margin + depth) * (self.width - self.Gpoint.y + 0.25) + margin)
            screen.blit(textG, (Gcenterx, Gcentery))
            for i in range(len(self.lPol)):
                for j in range(len(self.lPol[i])):
                    for k in range(len(self.lPol[i][j])):
                        for m in range(len(self.lPol[i][j][k])):
                            pygame.draw.rect(screen, color_Poly[i], [(margin + width) * int(self.lPol[i][j][k][0]) + margin,
                                                              (margin + depth) * (
                                                                  (self.width- int(self.lPol[i][j][k][1]))) + margin,
                                                              width,
                                                              depth])
            if self.path == []:
                pygame.time.wait(80)
                pygame.display.update()
            else:
                for i in range(len(self.path)):
                    if (int(self.path[i][0]) == self.Gpoint.x and int(self.path[i][1]) == self.Gpoint.y) or (int(self.path[i][0]) == self.Spoint.x and int(self.path[i][1]) == self.Spoint.y):
                        i+=1
                    else:
                        if (self.Ppoint != []):
                            check = 0
                            for j in range(len(self.Ppoint)):
                                if (int(self.path[i][0]) == self.Ppoint[j].getX() and int(self.path[i][1]) == self.Ppoint[j].getY()):
                                    check +=1
                            if check != 0:
                                i+=1
                            else:
                                if (self.path[i][2] > 1):
                                    pygame.draw.rect(screen, coincidence, [(margin + width) * int(self.path[i][0]) + margin,
                                                                          (margin + depth) * (
                                                                                      self.width - int(self.path[i][1])) + margin,
                                                                          width,
                                                                          depth])
                                else:
                                    pygame.draw.rect(screen, color_path, [(margin + width) * int(self.path[i][0]) + margin,
                                                                           (margin + depth) * (
                                                                                   self.width - int(
                                                                               self.path[i][1])) + margin,
                                                                           width,
                                                                           depth])
                        else:
                            pygame.draw.rect(screen, color_path, [(margin + width) * int(self.path[i][0]) + margin,
                                                                  (margin + depth) * (
                                                                          self.width - int(
                                                                      self.path[i][1])) + margin,
                                                                  width,
                                                                  depth])
                    pygame.time.wait(80)
                    pygame.display.update()
        pygame.display.flip()
    def PrintMap(self):
        print('Size Map', '\nDepth: ', self.depth, '\nWidth: ', self.width)
    def PrintSPoint(self):
        self.Spoint.PSPoint()
    def PrintGPoint(self):
        self.Gpoint.PGPoint()
    def PrintPickPoint(self):
        for i in range(len(self.Ppoint)):
            self.Ppoint[i].PPickupPoint()
    def PrintListPol(self):
        self.poly.PrintSerialPoly()

class Search:
    def __init__(self, map):
        self.map = map
        self.m = len(map.Ma)
        self.n = len(map.Ma[0])
        self.c = 0
        return
    def distance(self, path):
        if path == []:
            return 0
        s = 0
        for i in range(len(path)-1):
            x0 = path[i][0]
            y0 = path[i][1]
            x1 = path[i+1][0]
            y1 = path[i+1][1]
            d = math.sqrt(pow((x1-x0), 2) + pow((y1-y0), 2))
            s = s + d
        return s
    def mindistance(self, d):
        mind = d[0]
        minp = 0
        for i in range(len(d)):
            if mind > d[i] and d[i]!=0:
                mind = d[i]
                minp = i
        return (minp, mind)
    def condition(self, x, y):
        if (x <= 0): return False
        if (x >= y): return False
        return True
    def DFS(self, x, y, Gx, Gy, ):
        for k in range(4):
            t1 = x + offsetx[k]
            t2 = y + offsety[k]
            if t1 == Gx and t2 == Gy:
                return [(x, y)]
        if (self.map.Ma[x][y] == ''):
            self.map.Ma[x][y] = '1'
        for k in range(4):
            xn = x + offsetx[k]
            yn = y + offsety[k]
            if xn < self.m and xn > 0 and yn < self.n and yn > 0:
                if self.map.Ma[xn][yn] == '':
                    self.map.Ma[xn][yn] = '1'
                    result = self.DFS(xn, yn, Gx, Gy)
                    if (len(result) > 0):
                        result.append((x, y))
                        return result
        return []
    def BFS(self, S, G):
        m = self.m
        n = self.n
        Sx = S.x
        Sy = S.y
        Gx = G.x
        Gy = G.y
        queueX = [0] * 10000
        queueY = [0] * 10000
        trace = [0] * 10000
        result = [0] * 10000
        status = np.array([True] * m * n).reshape(m, n)
        dem = 1
        tmp = 1
        vt = 0
        queueX[0] = Sx
        queueY[0] = Sy
        status[Sx, Sy] = False
        Ma = self.map.Ma
        while (True):
            for i in range(vt, dem):
                for j in range(0, 4):
                    if (self.condition(queueX[i] + offsetx[j], m) and self.condition(queueY[i] + offsety[j], n)):
                        if (Ma[int(queueX[i] + offsetx[j])][int(queueY[i] + offsety[j])] == '#'):
                            continue
                        elif (status[queueX[i] + offsetx[j], queueY[i] + offsety[j]]):
                                status[queueX[i] + offsetx[j], queueY[i] + offsety[j]] = False
                                queueX[tmp] = queueX[i] + offsetx[j]
                                queueY[tmp] = queueY[i] + offsety[j]
                                trace[tmp] = i
                                tmp += 1
                                if (status[Gx, Gy] == False): break
                if (status[Gx, Gy] == False): break
            if dem == tmp: return []
            vt = dem
            dem = tmp
            if (status[Gx, Gy] == False): break
        d = 1
        result[0] = tmp - 1
        while (True):
            result[d] = trace[result[d - 1]]
            d += 1
            if (result[d - 1] == 0): break;

        pathX = [queueX[result[i]] for i in range(d - 1, -1, -1)]
        pathY = [queueY[result[i]] for i in range(d - 1, -1, -1)]
        ret = []
        for i in range(len(pathX))[::-1]:
            x = int(pathX[i])
            y = int(pathY[i])
            if (x == S.x and y == S.y) or (x == G.x and y == G.y):
                i-=1
            else:
                ret.append((x, y))
        return ret
    def checkXY(self, A, x, y, m, n):
        if (x <= 0 or x >= m or y <= 0 or y >= n):
            return 0
        v = A[x][y]
        if (v == '#'):
            return 0
        return 1
    def heuCal(self, g, x, y, gx, gy):
        h = abs(x - gx) + abs(y - gy)
        return g + h
    def AstPathFinding(self, pCheck, map, sx, sy, px, py):
        A = map.Ma
        limit = 50000
        m, n = int(map.depth), int(map.width)
        offSetX = np.array([0, 1, 0, -1])
        offSetY = np.array([1, 0, -1, 0])
        P = np.random.randint(1, size=(m, n))
        preX = np.random.randint(1, size=(m, n))
        preY = np.random.randint(1, size=(m, n))
        preX[sx][sy] = -1
        preY[sx][sy] = -1
        P = np.random.randint(1, size=(m, n))
        heu = np.random.randint(1, size=(m, n))
        for i in range(m):
            for j in range(n):
                heu[i][j] = limit
                P[i][j] = limit
        P[sx][sy] = 0
        heu[sx][sy] = self.heuCal(P[sx][sy], sx, sy, px, py)
        Q = Heap.Heap()
        Q.add(heu[sx][sy], sx, sy)
        notFound = 1
        while ((notFound) and Q.heap):
            D, u, v = Q.min(), Q.minX(), Q.minY()
            Q.del_min()
            if (u == px and v == py):
                notFound = 0
                continue
            if (heu[u][v] < D):
                continue
            for k in range(4):
                x = u + offSetX[k]
                y = v + offSetY[k]
                if (self.checkXY(A, x, y, m, n)):
                    h = self.heuCal(P[u][v] + 1, x, y, px, py)
                    if (h < heu[x][y]):
                        heu[x][y] = h
                        P[x][y] = P[u][v] + 1
                        Q.add(heu[x][y], x, y)
                        preX[x][y] = u
                        preY[x][y] = v
        if (notFound):
            pPath = None
            return pPath, pCheck
        while (Q.heap):
            Q.del_min()
        pPath = []
        # x, y = preX[px][py], preY[px][py]
        x, y = px, py
        while (preX[x][y] >= 0 and preY[x][y] >= 0):
            pointA = Point(x, y)
            pPath.append(pointA)
            pCheck[x][y] = 1
            u = preX[x][y]
            y = preY[x][y]
            x = u
        l = len(pPath)
        for i in range(int(l / 2)):
            pPath[i], pPath[l - i - 1] = pPath[l - i - 1], pPath[i]

        return pPath, pCheck

    def updatePath(self, pPath, fPath):
        if pPath == None:
            return None
        for i in range(len(pPath)):
            fPath.append(pPath[i])
        return fPath

    def updatePathExample(self, A, path, fPath):
        if fPath == None:
            return None
        # print(len(fPath),'len')
        for i in range(len(fPath)):
            u = fPath[i].getX()
            v = fPath[i].getY()
            if A[u][v] == '':
                path[u][v] = 1
    def Ast(self):
        path = self.map.Ma
        A = self.map.Ma
        limit = 50000
        fPath = []
        m, n = int(self.m), int(self.n)
        # offSetX = np.array([0, 1, 0, -1])
        # offSetY = np.array([1, 0, -1, 0])
        # P = np.random.randint(1, size=(m, n))
        # preX = np.random.randint(1, size=(m, n))
        # preY = np.random.randint(1, size=(m, n))
        heu = np.random.randint(1, size=(m, n))
        pCheck = np.random.randint(1, size=(m, n))

        Q = Heap.Heap()
        sx = int(self.map.Spoint.getX())
        sy = int(self.map.Spoint.getY())

        # preX[sx][sy] = -1
        # preY[sx][sy] = -1
        for i in range(m):
            for j in range(n):
                heu[i][j] = limit
                # P[i][j] = limit

        # P[sx][sy] = 0
        nP = len(self.map.Ppoint)
        # print(nP)
        gx = int(self.map.Gpoint.x)
        gy = int(self.map.Gpoint.y)
        # map.Ppoint
        notCheckP = np.random.randint(1, size=nP)
        notDone = 1
        if nP <= 0:
            notDone = 0
        count = 0
        xF = sx
        yF = sy
        maxD = np.random.randint(1, size=nP)
        while notDone:
            count = 0
            imax = 0

            for i in range(nP):
                maxD[i] = -limit
            for i in range(nP):

                z = self.map.getPpoint(i)
                u = z.getX()
                #v = z.getY()
                v = z.getY()

                # print(u,' ',v)
                if (pCheck[u][v]):
                    count = count + 1
                else:
                    maxD[i] = (abs(u - gx) + abs(v - gy)) - (abs(u - sx) + abs(v - sy))
                    if (maxD[i] > maxD[imax]):
                        imax = i

            if count == nP:
                notDone = 0
                continue

            z = self.map.getPpoint(imax)
            xF = z.getX()
            yF = z.getY()
            pCheck[xF][yF] = 1
            pPath, pCheck = self.AstPathFinding(pCheck, self.map, sx, sy, xF, yF)
            if pPath == None:
                break
            sx = xF
            sy = yF
            self.updatePath(pPath, fPath)
        if notDone:
            fPath = None
        else:
            pPath, pCheck = self.AstPathFinding(pCheck, self.map, xF, yF, gx, gy)
            self.updatePath(pPath, fPath)
            if pPath == None:
                path = self.map.Ma
                fPath = None
            else:
                self.updatePathExample(A, path, fPath)
        #path = self.map.Ma
        # printPath in mang thi du duong di
        return fPath

def ReadInput(fname):
    fin = open(fname, 'r')
    ar = []
    map = Map(0, 0)
    for line in fin:
        ar.append(line.strip(','))
    for i in range(len(ar)):
        if i == 0:
            p = ar[i].split(',')
            if len(p) > 2:
                return (map, False)
            map = Map(p[0], p[1])
            #map.PrintMap()
        if i == 1:
            p = ar[i].split(',')
            if (len(p) < 2 and len(p)/2 != 0):
                return (map, False)
            j = 0
            while j < len(p) - 1:
                if j < 1:
                    Spoint = Point(p[j], p[j + 1])
                    map.setSpoint(Spoint)
                    #map.PrintSPoint()
                if j > 1 and j <= 3:
                    Gpoint = Point(p[j], p[j + 1])
                    map.setGpoint(Gpoint)
                    #map.PrintGPoint()
                elif j > 3:
                    PickPoint = Point(p[j], p[j + 1])
                    map.setPickpoint(PickPoint)
                j += 2
        if i == 2:
            map.setPolyquantum(ar[i])
            #print('Polygon quantum: ', ar[i])
        if i > 2:
            j = 0
            p = ar[i].split(',')
            if (len(ar)-3 != int(ar[2])):
                return (map, False)
            Poly = Polygon(0)
            while j < len(p) - 1:
                point = Point(p[j], p[j + 1])
                Poly.pushPoint(point)
                j += 2
            map.setlistPol(Poly)
    #map.PrintListPol()
    #map.PrintPickPoint()
    return (map, True)
    #map.FindpathDFS()
    #map.Illustration('DFS')

def Execute(map, Algorithm):
    map.DrawP()
    if Algorithm == 'DFS':
        map.Illustration('DFS')
    elif Algorithm == 'BFS':
        map.Illustration('BFS')
    elif Algorithm == 'A*':
        map.Illustration('A*')

def main():
    filename = 'input.txt'
    Map, bool = ReadInput(filename)
    Algo = input("Enter the algorithm you want: ")
    if bool == True:
        Execute(Map, Algo)
    else:
        print('Invalid input! Please check your input file!')

if __name__ == '__main__':
    main()
