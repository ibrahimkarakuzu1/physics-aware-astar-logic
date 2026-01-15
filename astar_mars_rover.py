import heapq # A* öncelik kuyruğu 
import math

#  fizik motoru
MASS = 900.0 
GRAVITY = 3.721
FRICTION = 0.04 

def calculate_energy_cost(p1, p2):
    """iki nokta arasındkai enerji maliyetini hesaplar"""

    dist_2d = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    delta_h = p2[2] - p1[2]
    dist_3d = math.sqrt(dist_2d**2 + delta_h**2)

    if dist_3d == 0: return 0
    theta_rad = math.atan2(delta_h, dist_2d)

    #kuvvetler 
    gravity_force = MASS * GRAVITY * math.sin(theta_rad)
    friction_force = MASS * GRAVITY * math.cos(theta_rad) * FRICTION
    total_force = gravity_force + friction_force

    if total_force < 0: total_force = 0 # yokuş aşağı bedava enerji maliyeti yok
    # joule cinsinde maliyet
    return total_force * dist_3d


#        A*  algoritması

class Node:
    # haritadaki her piksel bir nodedur. 
    def __init__(self, x, y, z):
        self.x, self.y, self.z, =x, y, z
        self.g = float('inf')
        self.f = float('inf') 
        self.parent = None 

    def __lt__(self,other):
            return self.f < other.f
def heuristic(node, goal):
    """
    Sezgisel Fonksiyon(h) Kuş uçuşu 3D mesafe
    Algoritmanın hedefe yönelmesini sağlar
    """
    return math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2 + (node.z - goal.z)**2 )
    
def reconstruct_path(current_node):
    """Bitişten geriye doğru giderek yolu çizer."""
    path = []
    total_energy = current_node.g
    while current_node:
        path.append((current_node.x, current_node.y, current_node.z))
        current_node = current_node.parent
    return path[::-1], total_energy # Listeyi ters çevir
def a_star_search (terrain_map, start_coord, goal_coord):
    """
    terrain_map: {(x,y): z} sözlüğü Harita verisi
    """
        
    #Başlangıç ve Bitiş düğümleri 
    start_node = Node(start_coord[0], start_coord[1], terrain_map[start_coord])
    goal_node = Node(goal_coord[0], goal_coord[1], terrain_map[goal_coord])

    start_node.g = 0
    start_node.f = heuristic(start_node, goal_node)

    #incelencekler açık liste 
    open_list = []
    heapq.heappush(open_list, start_node)

    # Tekrar bakmamak için ziyaret edilenler
    visited = set()
        # Nodeları saklamak için sözlük yapısı
    nodes = {start_coord: start_node}

    print("\n Rover Navigasyon Sistemi Başlatıldı.. ")

    while open_list:
        current = heapq.heappop(open_list)

        if (current.x, current.y) == (goal_node.x, goal_node.y):
            print("HEDEFE VARILDII Rota hesaplanıyor....")
            return reconstruct_path(current)
            
        visited.add((current.x, current.y))

        # komşlara bak
        neighbors = [
            (0,1), (0,-1), (1,0), (-1,0), #düz
            (1,1), (1,-1), (-1,1), (-1,-1) #çapraz
        ]

        for dx, dy in neighbors:
            nx, ny = current.x + dx, current.y + dy

            #harita sınırları içerisinde mi
            if (nx, ny) in terrain_map:
                #daha önce buraya baktık mı 
                if (nx, ny) in visited:
                    continue
                    
                nz= terrain_map[(nx, ny)]

                # ENERJİ MALİYETİ 
                move_cost = calculate_energy_cost((current.x, current.y, current.z), (nx, ny, nz))

                new_g = current.g + move_cost

                # eğer bu kareye ilk defa geldiysek veya daha ucuz bir yol bulduysak 
                if (nx, ny) not in nodes or new_g < nodes[(nx, ny)].g:
                    neighbor = Node(nx, ny, nz)
                    neighbor.g = new_g
                    neighbor.f = new_g + heuristic(neighbor, goal_node)
                    neighbor.parent = current

                    nodes[(nz, ny)] = neighbor
                    heapq.heappush(open_list, neighbor)
    return None, 0    
    
    
if __name__ == "__main__":
    # Basit bir harita oluşturalım (5x5 Grid)
    # Ortada koca bir DAĞ var Yükseklik: 50m Kenarlar DÜZ 0m
    terrain = {}
    for x in range(5):
        for y in range(5):
            terrain[(x, y)] = 0 # Önce her yer düz
            
    # Ortaya Dağ Koyalım (Engel)
    terrain[(2, 2)] = 50 
    terrain[(2, 1)] = 30
    terrain[(1, 2)] = 30
    
    start = (0, 0)
    goal = (4, 4) # Dağın arkasında
    
    path, energy = a_star_search(terrain, start, goal)
    
    print("-" * 40)
    print(f" ROTA ANALİZİ:")
    print(f" Başlangıç: {start}")
    print(f" Hedef    : {goal}")
    print(f" Toplam Enerji: {energy:.2f} Joules")
    print(" İzlenen Yol (Koordinatlar ve Yükseklik):")
    for step in path:
        print(f"   -> {step}")



            




