import math


class DStar:
    def __init__(self, s_start, s_goal, x_range, y_range, obstacles):
        self.s_start = s_start
        self.s_goal = s_goal

        self.u_set = [  # 8-connected grid
            (1, 0), (0, 1), (-1, 0), (0, -1),
            (1, 1), (-1, 1), (-1, -1), (1, -1)
        ]
        self.obs = obstacles
        self.x_range = x_range
        self.y_range = y_range

        self.OPEN = set()
        self.t = dict()
        self.PARENT = dict()
        self.h = dict()
        self.k = dict()
        self.path = []
        self.visited = set()

    def init(self):
        for x in range(self.x_range):
            for y in range(self.y_range):
                self.t[(x, y)] = 'NEW'
                self.k[(x, y)] = 0.0
                self.h[(x, y)] = float("inf")
                self.PARENT[(x, y)] = None

        self.h[self.s_goal] = 0.0

    def run(self, s_start, s_end):
        self.init()
        self.insert(s_end, 0)

        while True:
            self.process_state()
            if self.t[s_start] == 'CLOSED':
                break

        self.path = self.extract_path(s_start, s_end)

    def extract_path(self, s_start, s_end):
        path = [s_start]
        s = s_start
        while True:
            s = self.PARENT[s]
            path.append(s)
            if s == s_end:
                return path

    def process_state(self):
        s = self.min_state()
        self.visited.add(s)

        if s is None:
            return -1

        k_old = self.get_k_min()
        self.delete(s)

        if k_old < self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.h[s_n] <= k_old and self.h[s] > self.h[s_n] + self.cost(s_n, s):
                    self.PARENT[s] = s_n
                    self.h[s] = self.h[s_n] + self.cost(s_n, s)

        if k_old == self.h[s]:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                   (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)) or \
                   (self.PARENT[s_n] != s and self.h[s_n] > self.h[s] + self.cost(s, s_n)):

                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
        else:
            for s_n in self.get_neighbor(s):
                if self.t[s_n] == 'NEW' or \
                   (self.PARENT[s_n] == s and self.h[s_n] != self.h[s] + self.cost(s, s_n)):

                    self.PARENT[s_n] = s
                    self.insert(s_n, self.h[s] + self.cost(s, s_n))
                else:
                    if self.PARENT[s_n] != s and \
                       self.h[s_n] > self.h[s] + self.cost(s, s_n):
                        self.insert(s, self.h[s])
                    elif self.PARENT[s_n] != s and \
                         self.h[s] > self.h[s_n] + self.cost(s_n, s) and \
                         self.t[s_n] == 'CLOSED' and \
                         self.h[s_n] > k_old:
                        self.insert(s_n, self.h[s_n])

        return self.get_k_min()

    def min_state(self):
        if not self.OPEN:
            return None
        return min(self.OPEN, key=lambda x: self.k[x])

    def get_k_min(self):
        if not self.OPEN:
            return -1
        return min([self.k[x] for x in self.OPEN])

    def insert(self, s, h_new):
        if self.t[s] == 'NEW':
            self.k[s] = h_new
        elif self.t[s] == 'OPEN':
            self.k[s] = min(self.k[s], h_new)
        elif self.t[s] == 'CLOSED':
            self.k[s] = min(self.h[s], h_new)

        self.h[s] = h_new
        self.t[s] = 'OPEN'
        self.OPEN.add(s)

    def delete(self, s):
        if self.t[s] == 'OPEN':
            self.t[s] = 'CLOSED'
        self.OPEN.remove(s)

    def modify(self, s):
        self.modify_cost(s)
        while True:
            k_min = self.process_state()
            if k_min >= self.h[s]:
                break

    def modify_cost(self, s):
        if self.t[s] == 'CLOSED':
            self.insert(s, self.h[self.PARENT[s]] + self.cost(s, self.PARENT[s]))

    def get_neighbor(self, s):
        nei_list = set()
        for u in self.u_set:
            s_next = (s[0] + u[0], s[1] + u[1])
            if 0 <= s_next[0] < self.x_range and 0 <= s_next[1] < self.y_range:
                if s_next not in self.obs:
                    nei_list.add(s_next)
        return nei_list

    def cost(self, s_start, s_goal):
        if self.is_collision(s_start, s_goal):
            return float("inf")
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        if s_start in self.obs or s_end in self.obs:
            return True

        # Optional: add extra diagonal checking logic here if needed
        return False
