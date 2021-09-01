import tkinter as tk
import random
import math
import numpy as np
import sys
import copy



waittime = 1
class Counter:
    def __init__(self,canvas):
        self.dirtCollected = 0
        self.canvas = canvas
        self.canvas.create_text(70,50,text="Dirt collected: "+str(self.dirtCollected),tags="counter")
        
    def itemCollected(self, canvas):
        self.dirtCollected +=1
        self.canvas.itemconfigure("counter",text="Dirt collected: "+str(self.dirtCollected))

    def itemCollected_draw(self, canvas,size):
        self.dirtCollected += size
        self.canvas.itemconfigure("counter",text="Dirt collected: "+str(self.dirtCollected))

class Collision_Counter:
    def __init__(self,canvas):
        self.collisions = 0
        self.canvas = canvas
        self.canvas.create_text(70,150,text="collisions occured: "+str(self.collisions),tags="counter")
        
    def itemCollected(self, canvas):
        self.collisions +=1
        self.canvas.itemconfigure("counter",text="Dirt collected: "+str(self.collisions))

class Charger:
    def __init__(self,namep, xp,yp):
        self.centreX = xp
        self.centreY = yp
        self.name = namep
        self.isAvailable = True
        
    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-10,self.centreY-10, \
                                  self.centreX+10,self.centreY+10, \
                                  fill="gold",tags=self.name)
        return body
        
    def getLocation(self):
        return self.centreX, self.centreY

    def update_Available (self, registryActives):
        temp = True
        for rr in registryActives:
            if rr.distanceTo(self)>30:
                 temp =  (temp and True)
        self.isAvailable = temp

def map_scaling (map_base, map_created):
    max = np.max(map_base)
    min = np.min(map_base)
    map_base_scaled = np.array([(x - min) / (max - min) for x in map_base])
    max = np.max(map_created)
    min = np.min(map_created)
    map_created_scaled = np.array([(x - min) / (max - min) for x in map_created])
    return map_base_scaled, map_created_scaled

class Comparer:
    def __init__(self,namep,map_base, map_created):
        map_base_scaled_created, map_created_scaled_created = map_scaling (map_base, map_created)
        self.name = namep
        self.map_base_scaled = map_base_scaled_created
        self.map_created_scaled = map_created_scaled_created

        self.base_mean = np.mean (map_base_scaled_created)
        self.created_mean = np.mean (map_created_scaled_created)
        
        self.base_std = np.std (map_base_scaled_created)
        self.created_std = np.std (map_created_scaled_created)

class Bot:

    def __init__(self,namep,canvasp,xx,yy,angle, range, map_grid):
        self.x = xx
        self.y = yy
        self.theta = angle
        self.name = namep
        self.ll = 60 #axle width
        self.vl = 0.0
        self.vr = 0.0
        self.battery = 4000
        self.canvas = canvasp
        self.cleaning_range = range
        self.map_x = map_grid.x
        self.w = map_grid.w #windows width
        self.map_y = map_grid.y
        self.h = map_grid.h #windows height
        self.move_away = 0
        self.map_visited = np.zeros((map_grid.x, map_grid.y),dtype=int)
        self.map_collected = np.zeros((map_grid.x, map_grid.y),dtype=int)
        self.charge_consumed = 0
        self.die_count = 0
        self.recharging = 0

    def draw(self,canvas):
        canvas.create_oval(self.x-30,self.y-30, \
                            self.x+30,self.y+30, \
                            fill="blue", tags = self.name) # covers dirt collection

        theta_45_front= math.sqrt(450) # at the circle edges, 45 degree from the rlative front
        self.sensorPositions = [ (self.x + theta_45_front*math.sin(self.theta)) + theta_45_front*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y - theta_45_front*math.cos(self.theta)) + theta_45_front*math.cos((math.pi/2.0)-self.theta), \
                                 (self.x - theta_45_front*math.sin(self.theta)) + theta_45_front*math.sin((math.pi/2.0)-self.theta), \
                                 (self.y + theta_45_front*math.cos(self.theta)) + theta_45_front*math.cos((math.pi/2.0)-self.theta)  \
                            ]
        
        centre1PosX = self.x 
        centre1PosY = self.y
        canvas.create_oval(centre1PosX-15,centre1PosY-15,\
                           centre1PosX+15,centre1PosY+15,\
                           fill="gold",tags=self.name)
        canvas.create_text(self.x,self.y,text=str(self.battery),tags=self.name)

        wheel1PosX = self.x - 30*math.sin(self.theta)
        wheel1PosY = self.y + 30*math.cos(self.theta)
        canvas.create_oval(wheel1PosX-3,wheel1PosY-3,\
                                         wheel1PosX+3,wheel1PosY+3,\
                                         fill="red",tags=self.name)

        wheel2PosX = self.x + 30*math.sin(self.theta)
        wheel2PosY = self.y - 30*math.cos(self.theta)
        canvas.create_oval(wheel2PosX-3,wheel2PosY-3,\
                                         wheel2PosX+3,wheel2PosY+3,\
                                         fill="green",tags=self.name)

        sensor1PosX = self.sensorPositions[0]
        sensor1PosY = self.sensorPositions[1]
        sensor2PosX = self.sensorPositions[2]
        sensor2PosY = self.sensorPositions[3]
        canvas.create_oval(sensor1PosX-3,sensor1PosY-3, \
                           sensor1PosX+3,sensor1PosY+3, \
                           fill="yellow",tags=self.name)
        canvas.create_oval(sensor2PosX-3,sensor2PosY-3, \
                           sensor2PosX+3,sensor2PosY+3, \
                           fill="yellow",tags=self.name)

    def return_to_charger(self,regPassive):
        #index = int (self.name [3:])
        #print(index)
        for rr in regPassive:
            if isinstance (rr, Charger):
                if rr.isAvailable:
                    self.x = rr.centreX
                    self.y = rr.centreY
                    break
        self.die_count +=1
        print(self.name, "died")
 
    # cf. Dudek and Jenkin, Computational Principles of Mobile Robotics
    def move(self,canvas,registryPassives,registryActives,dt, w,h):

        for rr in registryPassives:
            if isinstance(rr,Charger) and self.distanceTo(rr)<10: 
                self.battery += 10
                if self.recharging >= 0:# is recharging, dont move
                    self.vl = 0
                    self.vr = 0
                    self.recharging -= 10
                else:
                    self.recharging -= 9
        if self.battery>0:
            if (abs(self.vl) + abs(self.vr)) != 0 : # if not moving, not consuming charge
                self.battery -= 1
                self.charge_consumed +=1
        if self.battery==0:
            self.vl = 0
            self.vr = 0
            self.return_to_charger(registryPassives)

                #print (self.name, self.x, self.y, "movement",self.vl)
        
        if self.vl==self.vr:
            R = 0
        else:
            R = (self.ll/2.0)*((self.vr+self.vl)/(self.vl-self.vr))
        
        omega = (self.vl-self.vr)/self.ll
        ICCx = self.x-R*math.sin(self.theta) #instantaneous centre of curvature
        ICCy = self.y+R*math.cos(self.theta)
        m = np.matrix( [ [math.cos(omega*dt), -math.sin(omega*dt), 0], \
                        [math.sin(omega*dt), math.cos(omega*dt), 0],  \
                        [0,0,1] ] )
        v1 = np.matrix([[self.x-ICCx],[self.y-ICCy],[self.theta]])
        v2 = np.matrix([[ICCx],[ICCy],[omega*dt]])
        newv = np.add(np.dot(m,v1),v2)
        newX = newv.item(0)
        newY = newv.item(1)
        newTheta = newv.item(2)
        newTheta = newTheta%(2.0*math.pi) #make sure angle doesn't go outside [0.0,2*pi)
        self.x = newX
        self.y = newY
        self.theta = newTheta
        if self.vl==self.vr: # straight line movement
            self.x += self.vr*math.cos(self.theta) #vr wlog
            self.y += self.vr*math.sin(self.theta)
        #strict bounds
        
        self.updateMap()
        canvas.delete(self.name)
        self.draw(canvas)
        
    def distanceTo(self,obj):
        xx,yy = obj.getLocation()
        return math.sqrt( math.pow(self.x-xx,2) + math.pow(self.y-yy,2) )

    def distanceToRightSensor(self,lx,ly):
        return math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                          (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )

    def distanceToLeftSensor(self,lx,ly):
        return math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                          (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )       
    
    def senseCharger(self, registryPassives):
        lightL = 0.0
        lightR = 0.0
        for pp in registryPassives:
            if isinstance(pp,Charger):
                if pp.isAvailable :
                    lx,ly = pp.getLocation()
                    distanceL = math.sqrt( (lx-self.sensorPositions[0])*(lx-self.sensorPositions[0]) + \
                                         (ly-self.sensorPositions[1])*(ly-self.sensorPositions[1]) )
                    distanceR = math.sqrt( (lx-self.sensorPositions[2])*(lx-self.sensorPositions[2]) + \
                                         (ly-self.sensorPositions[3])*(ly-self.sensorPositions[3]) )
                    lightL += 200000/(distanceL*distanceL)
                    lightR += 200000/(distanceR*distanceR)
        return lightL, lightR
    
    def collectDirt(self, canvas, registryPassives, count):
        toDelete = []
        xMapPosition = int(math.floor(self.x/(self.w/self.map_x)))
        if xMapPosition >= self.map_x :
            xMapPosition = self.map_x-1 # incase if right at the edge
        
        yMapPosition = int(math.floor(self.y/(self.h/self.map_y)))
        if yMapPosition >= self.map_y :
            yMapPosition = self.map_y-1 # incase of right at the edge

        for idx,rr in enumerate(registryPassives):
            if isinstance(rr,Dirt):
                if self.distanceTo(rr)<30:
                    canvas.delete(rr.name)
                    toDelete.append(idx)
                    count.itemCollected(canvas)
                    self.map_collected[xMapPosition][yMapPosition] += 1
        for ii in sorted(toDelete,reverse=True):
            del registryPassives[ii]
        return registryPassives
    
    # change/add to complete communication
    def list_travel_generate (self, map_grid):
        list_travel =list()
        for xx in range (self.map_x):
            for yy in range( self.map_y):
                if (self.map_visited[xx][yy] != 1):
                    grid_center_x , grid_center_y = map_grid.grid_centers[xx][yy]
                    distance_from_self = math.sqrt( math.pow(grid_center_x-self.x,2) + math.pow(grid_center_y-self.y, 2))
                    list_travel.append ((xx, yy, distance_from_self))
        
        list_travel.sort (key= lambda x:x[2])
        return (list_travel)

    def transferFunction(self, map_grid, chargerL,chargerR, registryActives, registryPassives):
        list_travel = self.list_travel_generate (map_grid)
        if len(list_travel) != 0:
            (target_x , target_y, distance_to_it) = list_travel [0] # Future development, add time taken to actively attempt to each, longer = higher cost
            grid_centers = map_grid.grid_centers
            (lx, ly) = (grid_centers [target_x] [target_y])
            distL = self.distanceToLeftSensor(lx, ly)
            distR = self.distanceToRightSensor(lx, ly) 
        else: #exploration done, pepare to return to charger
            distL = chargerL
            distR = chargerR
            #print ("retreat")
            self.recharging = 100

        self.collision_active(registryActives, registryPassives)
        self.collision_passive(registryPassives, map_grid.w, map_grid.h)
        if self.recharging <= 0: # no longer needs rechatge
            if self.move_away >= 0: # is not keeping away
                self.move_away = self.move_away-1
                self.vl = random.randrange(0,5)
                self.vr = random.randrange(0,5)
            elif distR>distL:
                self.vl = 2.0
                self.vr = -2.0
            elif distR<distL:
                self.vl = -2.0
                self.vr = 2.0
            if abs(distR-distL)<distL*0.1: #approximately the same
                self.vl = 5.0
                self.vr = 5.0
        if self.battery<(1600) or self.recharging > 0: #approximate centre, from corner, i.e the furthest from any corner
            if (self.battery%100 == 0) and (self.recharging > 0) and (self.battery < 4000) :
                print(self.name,"going to recharge: ",self.battery)
            self.recharging = 4000-self.battery
            if chargerR>chargerL:
                self.vl = 2.0
                self.vr = -2.0
            elif chargerR<chargerL:
                self.vl = -2.0
                self.vr = 2.0
            if (abs(chargerR-chargerL)<chargerL*0.15): #approximately the same
                self.vl = 5.0
                self.vr = 5.0
            
    def updateMap(self):
        xMapPosition = int(math.floor(self.x/(self.w/self.map_x)))
        if xMapPosition < 0 :
            xMapPosition = 0
        if xMapPosition >= self.map_x :
            xMapPosition = self.map_x-1 # incase if right at the edge
        yMapPosition = int(math.floor(self.y/(self.h/self.map_y)))
        if yMapPosition >= self.map_y :
            yMapPosition = self.map_y-1 # incase of right at the edge
        if yMapPosition < 0 :
            yMapPosition = 0
        self.map_visited[xMapPosition][yMapPosition] = 1
        #print(self.name, xMapPosition, yMapPosition )

    def init_map (self, reg_passive): # declare charger zones as already visited
        for rp in reg_passive:
            if isinstance (rp, Charger):
                c_x, c_y = rp.getLocation()
                xMapPosition = int(math.floor(c_x/(self.w/self.map_x)))
                if xMapPosition >= self.map_x :
                    xMapPosition = self.map_x-1 # incase if right at the edge
                yMapPosition = int(math.floor(c_y/(self.h/self.map_y)))
                if yMapPosition >= self.map_y :
                    yMapPosition = self.map_y-1 # incase of right at the edge
                self.map_visited[xMapPosition][yMapPosition] = 1
                #print(xMapPosition,",",yMapPosition)


    def drawMap(self):
        intervals_x = self.w/self.map_x
        intervals_y = self.h/self.map_y
        for xx in range(0,self.map_x):
            for yy in range(0,self.map_y):
                if self.map_visited[xx][yy]==1:
                    #print(xx,",",yy,)
                    self.canvas.create_rectangle(intervals_x*xx,intervals_y*yy,intervals_x*xx+intervals_x,intervals_y*yy+intervals_y,fill="pink",width=0,tags="map") 
                    # bugged? ^, canvas none existent?, display bug only
        self.canvas.tag_lower("map")
    
    def collision_active (self, regActive, regPassive):
        for rr in regActive :
            if rr.name == self.name:
                continue
            else :
                if math.sqrt(((rr.x-self.x) * (rr.x-self.x)) + ((rr.y - self.y ) * (rr.y-self.y)) ) <= (self.cleaning_range * 2) :
                    if (rr.x-self.x) != 0:
                        tangent_angle = -(math.pi/2)+np.arctan( (rr.y - self.y ) /(rr.x-self.x))
                    else:
                        tangent_angle = math.pi /2 * (1 if (rr.y - self.y) > 0 else -1)
#                    self.theta = (random.randrange(0,10)/20 * math.pi)  + tangent_angle
                    self.theta = (self.theta * -1)  + (2* tangent_angle)  #reflection style bounce
                    self. move_away = random.randrange(5,15)
#                    self.die_count +=1
#                else:
#                    continue
    def collision_passive (self, regPassive, w, h):
        if (self.x<0.0 or self.x>w):
            self.theta = ((self.theta * -1) + math.pi) %(2.0*math.pi)
            self. move_away = random.randrange(1,3)
            if self.theta == (math.pi /2 ) :
                self.theta = 0.49 * math.pi
            if self.theta == -(math.pi /2 ) :
                self.theta = -0.49 * math.pi
        if (self.y<0.0 or self.y>h):
            self.theta = (self.theta * -1) %(2.0*math.pi)
            self. move_away = random.randrange(1,3)
            if self.theta == 0 :
                self.theta = 0.1 * math.pi
            if self.theta == math.pi:
                self.theta = 0.9 * math.pi
        if (self.x<0.0):
            self.x = 0.0
        if (self.x>w):
            self.x = w
        if (self.y<0.0):
            self.y = 0.0
        if (self.y>h):
            self.y = h
        
class Dirt:
    def __init__(self,namep,xx,yy):
        self.centreX = xx
        self.centreY = yy
        self.name = namep

    def draw(self,canvas):
        body = canvas.create_oval(self.centreX-1,self.centreY-1, \
                                  self.centreX+1,self.centreY+1, \
                                  fill="grey",tags=self.name)

    def getLocation(self):
        return self.centreX, self.centreY

class Map:
    def __init__(self, namep, w, h, c_range):
        self.name = namep
        min_area_collect = (math.pi * c_range*c_range) # min area of quarter circle
        #to ensure min_area cover over half a grid area  min_area= grid_area/2
        min_grid_area = min_area_collect
        #if radius match
        #min_grid_side_length = 2*(math.sqrt(math.pow(c_range,2))/2)
        #min_grid_side_length = math.sqrt (min_grid_area)
        min_grid_side_length = 2*c_range
        self.x = int (math.floor(w/ min_grid_side_length)) + 1
        print("grid", self.x)
        self.w = w
        self.y = int (math.floor(h/ min_grid_side_length)) + 1
        self.h =h 
        self.grid_visited = np.zeros ((self.x,self.y), dtype=np.int16)
        self.grid_centers = self.grid_center_create(w,h)

    def grid_center_create(self, w,h):
        intervals_x = w/self.x
        top_l_corner_center_x = intervals_x/2
        intervals_y = h/self.y
        top_l_corner_center_y = intervals_y/2
        grid_centers = np.empty((self.x, self.y), "f,f")
        for x in range (self.x):
            for y in range(self.y):
                grid_centers[x][y] = ((top_l_corner_center_x + (x * intervals_x)), (top_l_corner_center_y + (y * intervals_y)))
        return grid_centers
    
class Wall_rectangle: # Future development
    def __init__(self, tag, w, h, theta):
        self.name = tag
        self.width = w #length
        self.height = h #width
        self.angle= theta

    def collision_angle (self, agent):
        print ("to be implemented") # in case of actual mapping of environment, instead of mapping pseudo heat map of dirt being gathered
        return False

#def buttonClicked(x,y,registryActives):
#    for rr in registryActives:
#        if isinstance(rr,Bot):
#            rr.x = x
#            rr.y = y

def initialise(window, w, h):
    window.resizable(False,False)
    canvas = tk.Canvas(window,width=w,height=h)
    canvas.pack()
    return canvas

def placeDirt(registryPassives,canvas, map_grid, w, h):
    #places dirt in a specific configuration
    map_x = map_grid.x
    map_y = map_grid.y

    intervals_x = w/map_x
    intervals_y = h/map_y

    area = intervals_x * intervals_y

    min_density = 1/100
    avg_dust_per_grid = min_density * area
    min_dust_per_grid = int(0.9*avg_dust_per_grid)
    #print(min_dust_per_grid)
    max_dust_per_grid = int (1.1 * avg_dust_per_grid)
        
    map = np.zeros( (map_x ,map_y), dtype=np.int16)
    for xx in range(map_x):
        for yy in range(map_y):
                map[xx][yy] = random.randrange(min_dust_per_grid,max_dust_per_grid)
    
    select_x_range = math.floor (map_grid.x / math.sqrt(10))
    select_y_range = math.floor (map_grid.y / math.sqrt(10))
    select_x = random.randrange (0, (map_x - select_x_range))
    select_y = random.randrange (0, (map_y - select_y_range))

    for xx in range(select_x_range):
        for yy in range(select_y_range):
                map[(xx + select_x)][(yy + select_y)] = random.randrange((5*min_dust_per_grid),(5*max_dust_per_grid))
    i = 0
    for xx in range(map_x):
        for yy in range(map_y):
            for _ in range(map[xx][yy]):
                dirtX = xx*intervals_x + random.randrange(0,math.floor(intervals_x))
                dirtY = yy*intervals_y + random.randrange(0,math.floor(intervals_y))
                dirt = Dirt("Dirt"+str(i),dirtX,dirtY)
                registryPassives.append(dirt)
                dirt.draw(canvas)
                i += 1
    #print(np.transpose(map))
    return map

def register(canvas,w ,h, c_range):
    registryActives = []
    registryPassives = []
    registryStorage = []
    charger= Charger("Charger"+str(1), 0,0)
    registryPassives.append(charger)
    charger.draw(canvas)
    charger= Charger("Charger"+str(2), w,h)
    registryPassives.append(charger)
    charger.draw(canvas)

    map_grid= Map ("map1", w, h, c_range)
    
    map_gen = placeDirt(registryPassives,canvas, map_grid, w, h)

    i=1
    for rr in registryPassives:
        if isinstance(rr, Charger):
            c_x, c_y = rr.getLocation()
            bot = Bot("Bot"+str(i),canvas,c_x ,c_y, random.vonmisesvariate(0,0), c_range, map_grid)
            registryActives.append(bot)
            bot.draw(canvas)
            i+=1

    for ra in registryActives:
        ra.init_map (registryPassives)

    count = Counter(canvas)
    #canvas.bind( "<Button-1>", lambda event: buttonClicked(event.x,event.y,registryActives) )
    for rr in registryPassives:
        if isinstance(rr, Charger):
            c_x, c_y = rr.getLocation()
            bot = Bot("Bot_storage"+str(i),canvas,c_x ,c_y,-3.0*math.pi/4.0, 0, map_grid)
            registryStorage.append(bot)
            i+=1
    return registryActives, registryPassives, count, map_gen, map_grid, registryStorage

def pseudo_or (a,b):
    ans= np.add(a,b)
    (r,c) =np.shape(ans)
    for xx in range (r):
        for yy in range (c):
            if ans[xx][yy] >= 1:
                ans[xx][yy]=1
    return ans

def moveIt(canvas,registryActives,registryPassives,reg_storage,count,window, w, h , map_gen, map_grid, percentage ):
    percent_explored = 0.0
    for rp in registryPassives:
        if isinstance (rp, Charger):
            rp.update_Available(registryActives)
    for rr in registryActives:
        chargerIntensityL, chargerIntensityR = rr.senseCharger(registryPassives)
        rr.transferFunction(map_grid, chargerIntensityL, chargerIntensityR,registryActives,registryPassives)
        rr.move(canvas,registryPassives,registryActives,1.0, w, h)
        registryPassives = rr.collectDirt(canvas,registryPassives, count)
        percent_explored += np.sum(rr.map_visited)
    #secondary loop, just to draw
    if percentage < percent_explored:
        percentage = percent_explored
        if (percentage% 20 == 0):
            print("percent", percent_explored / (2*rr.map_x *rr.map_y))

    if percentage == (2*rr.map_x *rr.map_y): 
        for idx_final, rr_final in enumerate(registryActives):
            bot_results = Bot("Bot_result_holder", canvas,0 ,0, 0, 0, map_grid) # not included in registry, simply to old information
            for rs in reg_storage:# pull out all of most recent information, create into a pseudo blackboard of places visited
                bot_results.map_visited = pseudo_or (rs.map_visited, bot_results.map_visited)
            new_agent_collected = rr_final.map_collected #functionally no different, is dependant on agents receiving communication

            reg_storage[idx_final].map_collected = copy.deepcopy(new_agent_collected)
            reg_storage[idx_final].die_count = copy.deepcopy(rr_final.die_count)
            reg_storage[idx_final].map_visited = copy.deepcopy(rr_final.map_visited) #updated storage, includes your own data, that was just gathered
        window.destroy()
    canvas.after(waittime,moveIt,canvas,registryActives,registryPassives, reg_storage, count,window,w, h , map_gen, map_grid, percentage )

def moveIt_comm(canvas,registryActives,registryPassives, reg_storage,count,window, w, h , map_gen, map_grid, percentage):
    percent_explored = 0.0
    for rp in registryPassives:
        if isinstance (rp, Charger):
            rp.update_Available(registryActives)
    for idx, rr in enumerate(registryActives):
        bot_results = Bot("Bot_result_holder", canvas,0 ,0, 0, 0, map_grid) # not included in registry, simply to old information
        chargerIntensityL, chargerIntensityR = rr.senseCharger(registryPassives)
        rr.transferFunction(map_grid, chargerIntensityL, chargerIntensityR,registryActives,registryPassives)
        rr.move(canvas,registryPassives,registryActives,1.0, w, h)
        registryPassives = rr.collectDirt(canvas,registryPassives, count)

        if rr.recharging >=0 and rr.battery > 1600: # change to include bool value in Bot object?, later
            for rs in reg_storage:# pull out all of most recent information, create into a pseudo blackboard of places visited
                bot_results.map_visited = pseudo_or (rs.map_visited, bot_results.map_visited)
            new_agent_collected = rr.map_collected #functionally no different, is dependant on agents receiving communication
            new_agent_visited = pseudo_or (rr.map_visited, bot_results.map_visited) #updated storage, includes your own data, that was just gathered

            reg_storage[idx].map_collected = copy.deepcopy(new_agent_collected)
            reg_storage[idx].die_count = copy.deepcopy(rr.die_count)
            reg_storage[idx].map_visited = copy.deepcopy(new_agent_visited) #updated storage, includes your own data, that was just gathered
        
            rr.map_visited = copy.deepcopy(reg_storage[idx].map_visited) # update agents map, base on aggregate data

        percent_explored += (np.sum(rr.map_visited) ) # If one of the aagents have complete mapping data, then only end (i.e. send end signal)
        
    if percentage < percent_explored:
        percentage = percent_explored
        if (percentage% 20 == 0):
            print("percent", percent_explored / (2*rr.map_x *rr.map_y))

    if percentage == (2*rr.map_x *rr.map_y):
        for idx_final, rr_final in enumerate(registryActives): # could be simplified into function; should I?
            bot_results = Bot("Bot_result_holder", canvas,0 ,0, 0, 0, map_grid) # not included in registry, simply to old information
            for rs in reg_storage:# pull out all of most recent information, create into a pseudo blackboard of places visited
                bot_results.map_visited = pseudo_or (rs.map_visited, bot_results.map_visited)
            new_agent_collected = rr_final.map_collected #functionally no different, is dependant on agents receiving communication
            new_agent_visited = pseudo_or (rr_final.map_visited, bot_results.map_visited) #updated storage, includes your own data, that was just gathered

            reg_storage[idx_final].map_collected = copy.deepcopy(new_agent_collected)
            reg_storage[idx_final].die_count = copy.deepcopy(rr_final.die_count)
            reg_storage[idx_final].map_visited = copy.deepcopy(new_agent_visited) #updated storage, includes your own data, that was just gathered

            rr_final.map_visited = copy.deepcopy(reg_storage[idx_final].map_visited) # update agents map, base on aggregate data

        window.destroy()
    
    canvas.after(waittime,moveIt_comm,canvas,registryActives,registryPassives, reg_storage,count,window, w, h , map_gen, map_grid, percentage )

def moveIt_complete_comm(canvas,registryActives,registryPassives, reg_storage,count,window, w, h , map_gen, map_grid,percentage):
    percent_explored = 0.0
    for rp in registryPassives:
        if isinstance (rp, Charger):
            rp.update_Available(registryActives)
        
    for idx, rr in enumerate(registryActives):
        bot_results = Bot("Bot_result_holder", canvas,0 ,0, 0, 0, map_grid) # not included in registry, simply to old information
        chargerIntensityL, chargerIntensityR = rr.senseCharger(registryPassives)
        rr.transferFunction(map_grid, chargerIntensityL, chargerIntensityR,registryActives,registryPassives)
        rr.move(canvas,registryPassives,registryActives,1.0, w, h)
        registryPassives = rr.collectDirt(canvas,registryPassives, count)
        for rs in reg_storage:# pull out all of most recent information, create into a pseudo blackboard of places visited
            bot_results.map_visited = pseudo_or (rs.map_visited, bot_results.map_visited)
        new_agent_collected = rr.map_collected
        new_agent_visited = pseudo_or(rr.map_visited, bot_results.map_visited) #updated storage, includes your own data, that was just gathered

        reg_storage[idx].map_collected = copy.deepcopy(new_agent_collected)
        reg_storage[idx].die_count = copy.deepcopy(rr.die_count)
        reg_storage[idx].map_visited = copy.deepcopy(new_agent_visited) #updated storage, includes your own data, that was just gathered
        
        rr.map_visited = copy.deepcopy(reg_storage[idx].map_visited) # update agents map, base on aggregate data

        percent_explored += np.sum(new_agent_visited)
    if percentage < percent_explored:
        percentage = percent_explored
        if (percentage% 20 == 0):
            print("percent", percent_explored / (2*rr.map_x *rr.map_y))

    if percentage == (2*rr.map_x *rr.map_y):
        window.destroy()
    
    canvas.after(waittime,moveIt_complete_comm,canvas,registryActives,registryPassives, reg_storage,count,window, w, h , map_gen, map_grid, percentage)


def single_run(info, run): # case 0 = independant, case 1 = recharge & debrief, case 2 = complete shared information
    width = 1000
    height = 1000
    collection_range = 30
    percentage = 0
    if info == 0 :
        run_type = "No communication"
    elif info == 1:
        run_type = "Limited communication"
    elif info ==2 :
        run_type = "Complete communication"
    else :
        run_type = "Wrong input"
    print("type: ",run_type)
    window = tk.Tk()
    canvas = initialise(window, width, height)
    
    registryActives, registryPassives, count, map_gen, map_grid, registryStorage = register(canvas, width, height, collection_range)
    bot_processor = Bot("run_result_processor", canvas,0 ,0,0, 0, map_grid) # not included in registry, simply to old information

    if info == 0 :
        moveIt(canvas,registryActives,registryPassives, registryStorage, count, window, width, height , map_gen, map_grid, percentage)
    elif info == 1:
        moveIt_comm(canvas,registryActives,registryPassives, registryStorage, count, window, width, height , map_gen, map_grid, percentage)
    elif info ==2 :
        moveIt_complete_comm(canvas,registryActives,registryPassives, registryStorage, count, window, width, height , map_gen, map_grid, percentage)    
    else: 
        print ("Wrong number input")
        window.destroy()
        return 0 ,0 , []
    window.mainloop()
    
    overall_charge_consumed = 0 
    overall_batterydeaths = 0

    for rr in registryActives:
        overall_charge_consumed += rr.charge_consumed
        overall_batterydeaths += rr.die_count
    for rs in registryStorage:
        #bot_processor.map_visited = pseudo_or (bot_processor.map_visited, rs.map_visited) # used for time limited analysis
        bot_processor.map_collected = copy.deepcopy(np.add(bot_processor.map_collected, rs.map_collected))

    results = Comparer ("Comparer run"+ str(run +1)+"type:"+run_type,map_gen, bot_processor.map_collected)
    print("charge consumed",overall_charge_consumed)
    print("battery_die count", overall_batterydeaths)
    print("Comparer created", results.name)
    return overall_charge_consumed,overall_batterydeaths, results

def data_process_mult (samples, info):
    overall_charge_consumed_list = []
    overall_batterydeaths_list = []
    comparer_list = []

    for x in range(samples):
        print("Run:",x +1)
        charge_single, deaths, res =single_run(info, x)
        overall_charge_consumed_list.append(charge_single)
        overall_batterydeaths_list.append(deaths)
        comparer_list.append (res)
    return overall_charge_consumed_list, overall_batterydeaths_list, comparer_list

def main():
    sample_size = 10
    charge = []
    deaths = []
    comparer_list = []

    for x in range (3):
        print("sample size: " , sample_size)
        c, death_list , dev= data_process_mult (sample_size, x)
        charge.append(c)
        deaths.append(death_list)
        comparer_list.append(dev)
    return charge, deaths, comparer_list

#charge, deaths, comparer_list = main()

#for x in range (0,11):
#    np.savetxt(("No_comm_"+str(x)+".csv"),np.subtract(comparer_list[0][x].map_base_scaled, comparer_list[0][x].map_created_scaled), delimiter = ",")
#for x in range (0,11):
#    np.savetxt(("Lim_comm_"+str(x)+".csv"),np.subtract(comparer_list[1][x].map_base_scaled, comparer_list[1][x].map_created_scaled), delimiter = ",")
#for x in range (0,11):
#    np.savetxt(("Full_comm_"+str(x)+".csv"),np.subtract(comparer_list[2][x].map_base_scaled, comparer_list[2][x].map_created_scaled), delimiter = ",")
