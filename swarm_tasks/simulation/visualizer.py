from swarm_tasks.simulation.simulation import Simulation
import swarm_tasks.envs as envs

import matplotlib.patches as patches
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
from shapely.geometry import Point
class Gui:
	def __init__(self, sim):
		self.sim = sim
		self.size = sim.size
		self.env_name = sim.env_name
		#self.ax = plt.axes()
		#self.fig = plt.gcf()
		self.fig, self.ax = plt.subplots(figsize=(10,10))
		self.ax.set_ylim([0, sim.size[1]])
		self.ax.set_xlim([0, sim.size[0]])

		#self.state_colors = ['blue','green','red','orange', 'purple']
		self.state_colors = ['b','g','r','c', 'm','y','purple','orange','k','brown','lime','pink','teal','gold','gray','violet']
		self.grid_scatter = None

		#Contents list (used in remove_artists())
		self.content_fills = []
		self.limits = ([0,0,self.size[0], self.size[0]], [0,self.size[1], self.size[1],0])
		self.area_covered=0
		self.search_time=0
		self.coverage_text = None
		self.coverage_text1 = None
		self.uav_trajectories=[]
		self.uav_positions = {'uav_1': [], 'uav_2': [], 'uav_3': [], 'uav_4': [], 'uav_5': [], 'uav_6':[], 'uav_7':[], 'uav_8':[]}
		
		
	def show_bots(self):

		#show bots
		for i,bot in enumerate(self.sim.swarm):
			#self.show_neighbourhood(bot,3)

			x,y,theta = bot.get_pose()
			state_disp = (bot.get_state()<=len(self.state_colors))*(bot.get_state())
			bot_color = self.state_colors[i % len(self.state_colors)]
			circle = plt.Circle((x,y), bot.size, color=bot_color, fill=True)
			plt.plot(marker='o', label='uav', color=self.state_colors)
			#circle = plt.Circle((x,y), bot.size, color=self.state_colors[state_disp], fill=True)
			self.fig.gca().add_artist(circle)

			l=0.15
			self.ax.arrow(x,y, \
				(bot.size-l)*np.cos(theta), (bot.size-l)*np.sin(theta), \
				head_width=l, head_length=l, \
				fc='k', ec='k', zorder=100)

	def show_env(self):
	    for obs in self.sim.env.obstacles:
	    	x, y = obs.exterior.xy
	    	self.ax.fill(x, y, fc='gray', alpha=0.9)
	    	if(self.env_name=="rectangles1"):
	    		x_coords = [int(i) for i in x]
		    	y_coords = [int(i) for i in y]

			# Set the grid cells inside the obstacle to True
		    	for i in range(max(0, min(x_coords)), min(self.sim.size[0], max(x_coords))):
		    		for j in range(max(0, min(y_coords)), min(self.sim.size[1], max(y_coords))):
		    			if 0 <= i < self.sim.size[1] and 0 <= j < self.sim.size[0]:
		    				if obs.contains(Point(i, j)):  # Check if the point is inside the polygon
		    					self.sim.grid[j, i] = True

	'''
	def show_env(self):
		for obs in self.sim.env.obstacles:
			x,y = obs.exterior.xy
			self.ax.fill(x,y, fc='gray', alpha=0.9)
			
			x_coords = [int(i) for i in x]
			y_coords = [int(i) for i in y]

			# Set the grid cells inside the obstacle to True
			for i in range(max(0, min(x_coords)), min(self.sim.size[0], max(x_coords) )):
		    		for j in range(max(0, min(y_coords)), min(self.sim.size[1], max(y_coords) )):
		    			if obs.contains(Point(i, j)):  # Check if the point is inside the polygon
		    				self.sim.grid[j, i] = True
			    				
			
			x_coords = [int(i) for i in x]
			y_coords = [int(i) for i in y]

			# Set the grid cells inside the obstacle to True
			for i in range(self.sim.size[0]):
			    for j in range(self.sim.size[1]):
			    	if (
				    min(x_coords) >= i >= max(x_coords)
				    and min(y_coords) >= j >= max(y_coords)
				    and obs.contains(Point(i, j))):
				    	self.sim.grid[j, i] = True
	'''

	def show_contents(self):
		for item in self.sim.contents.items:
			x,y = item.polygon.exterior.xy
			if item.subtype == 'contamination':
				self.ax.fill(x,y, fc='red', alpha=0.5)
				continue
			elif item.subtype == 'nest':
				self.ax.fill(x,y, fc='purple', alpha =0.3)
				continue
			self.ax.fill(x,y, fc='orange', alpha=0.9)
			#self.content_fills.append((x,y))
	

	def update(self):
		"""

		"""
		self.remove_artists()
		#self.show_bots()
		if self.sim.has_item_moved:
			#for ext in self.content_fills:
				#self.ax.fill(*ext, fc='w')
			#self.content_fills=[]
			#self.ax.fill(*self.limits, 'w')
			self.show_contents()
			self.show_env()
			self.sim.has_item_moved = False
		self.show_bots()
		if(self.env_name=="rectangles1"):
			self.show_coverage(self.area_covered,self.search_time)
		plt.pause(0.0005)

	def remove_artists(self):
		"""
		Previous figures need to be removed before updating positions on GUI
		"""
		for obj in self.ax.findobj(plt.Circle):
			obj.remove()

		for obj in self.ax.findobj(patches.FancyArrow):
			obj.remove()
		
		if self.sim.has_item_moved:
			for obj in self.ax.findobj(patches.Polygon):
				obj.remove()


	def show_neighbourhood(self, bot, r=None):
		#Shows the neighbourhood of a robot
		x,y = bot.get_position()
		if r == None:
			r = bot.neighbourhood_radius
		circle2 = plt.Circle((x,y), r, color='red', fill=True, alpha=0.1)
		circle1 = plt.Circle((x,y), r/2+bot.size, color='red', fill=True, alpha=0.1)
		self.fig.gca().add_artist(circle1)
		self.fig.gca().add_artist(circle2)


	def show_grid(self):
		"""
		Args:
			grid: A 2D array of values from 0-1 
		
		TODO:
		Plotting the grid as an image takes high computation
		Plot in another format
		"""
		if self.grid_scatter != None:
			self.grid_scatter.remove()

		x_size, y_size = self.sim.size
		x_labels = [i +0.5 for i in range(x_size)]*y_size
		y_labels = [int(i/x_size)+0.5 for i in range(y_size*x_size)]
		
		grid_1d = self.sim.grid.ravel()
		#grid_1d[20:40] = True	#debug

		col = ['green' if i else 'yellow' for i in grid_1d]


		if len(col) != len(x_labels) or len(x_labels) != len(y_labels):
			print("ERROR: Grid shape mismatch")
			return False

		self.grid_scatter = self.ax.scatter(x_labels, y_labels, c=col)
		#plt.draw()

	def show_coverage(self, area_covered,search_time):
		self.area_covered=area_covered
		self.search_time=search_time
		#self.elapsed_time_value = elapsed_time
		timer_text = f"Time: {int(self.search_time // 60):02d}:{int(self.search_time % 60):02d}"
		if self.coverage_text is not None:
			self.coverage_text.remove()			
			#if hasattr(self, 'elapsed_time_text'):
			 #   self.elapsed_time.remove()
		if self.coverage_text1 is not None:
			self.coverage_text1.remove()	
		# Adjust the coordinates as needed for the position of the text box
		text_box_props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
		label_text = "Area Covered ="
		self.coverage_text = self.ax.text(1, 1.05, f"{label_text} {area_covered}", transform=self.ax.transAxes,
		                  fontsize=10, verticalalignment='top', horizontalalignment='right', bbox=text_box_props)
            
            
		text_box_props1 = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
		label_text1 = "Time ="
		self.coverage_text1 = self.ax.text(1, 1.09, f"{timer_text} ", transform=self.ax.transAxes,
		                  fontsize=10, verticalalignment='top', horizontalalignment='right', bbox=text_box_props1)
        
		return area_covered
	def run(self):
		plt.show(block=False)
		
	def close(self):
		plt.close()
		
