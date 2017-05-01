#!/usr/bin/python
import math
import random

import actionlib
import rospy
import tf
import math
import random
import time as timer
from CostCell import CostCell
from nav_msgs.msg import GridCells, Odometry
from std_msgs.msg import Bool
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan

# Explorer class for frontier navigation
class Explorer(object):

	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.exp_cells = []
		self.wall_cells = []
		self.path_cells = []
		self.front_cells = []
		self.map_res = .29 # default resolution
		self.cost_grid = []
		self.odom_list = None # transform listener
		self.move = None # move_base publisher
		self.done = False
		self.at_goal = True
		self.unreachable = False # unreachable goal detected
		self.new_goal = None
		self.flame_seen = False
		self.map_ox = 0
		self.map_oy = 0

		self.last_laser = None
		self.last_laser_dist = 0.0
		self.laser_th = None
		self.odom_x = 0
		self.odom_y = 0
		self.odom_th = 0

		self.cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
		self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=50)
		self.fan_pub = rospy.Publisher('/toggle_fan', Bool, queue_size=1)
		self.ard_sub = rospy.Subscriber('/ard_odom', Twist, self.ard_sub_callback, queue_size=1)
		self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_sub_callback, queue_size=1)

		self.exp_pub = rospy.Publisher('/expanded_cells', GridCells, queue_size=1)
		self.path_pub = rospy.Publisher('/path_cells', GridCells, queue_size=1)
		self.front_pub = rospy.Publisher('/frontier_cells', GridCells, queue_size=1)
		self.wall_pub = rospy.Publisher('/wall_cells', GridCells, queue_size=1)

		self.flame_pub = rospy.Publisher('/flame_seen', Bool, queue_size=1)
		self.last_goal = None # previous goal

	def get_centroid(self, front):
		# get the centroid of a frontier
		x_sum =0.0
		y_sum = 0.0
		# sum
		for cell in front:
			x_sum = x_sum + cell.get_x_pos()
			y_sum = y_sum + cell.get_y_pos()

		# divide
		cell_x = int(x_sum / len(front))
		cell_y = int(y_sum / len(front))

		cell = self.cost_grid[cell_x][cell_y]

		if not cell.is_empty() or cell.is_unknown():
			cell = self.approx(cell)

		return cell

	def turn360(self, time):
		self.vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,(2.0*math.pi)/time)))
		timer.sleep(time)
		self.vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def stop_motors(self):
		self.vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def backup(self, dist):
		self.vel_pub.publish(Twist(Vector3(-1*dist,0,0),Vector3(0,0,0)))
		timer.sleep(1)
		self.vel_pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	def get_candle_xy(self):
		len_scan = len(self.last_laser)
		last_dist = 1000
		pos = 0

		rospy.logwarn("Length of laserscan: {}".format(last_dist))
		for i in range(len_scan):
			if self.last_laser[i] < last_dist:
				last_dist = self.last_laser[i]
				pos = i
		#candle_th = (pos-20)*self.laser_th + self.odom_th

		# if the candle is too far use 18 inches (0.4572 m)
		# if last_dist > 0.7:
		# 	last_dist = 0.3

		self.last_laser_dist = last_dist

		candle_x = last_dist+0.05 * math.cos(self.z) + self.odom_x
		candle_y = last_dist+0.05 * math.sin(self.z) + self.odom_y

		return [candle_x, candle_y]

	def go_home(self):
		goal = PoseStamped()
		goal.header.frame_id = 'map'
		goal.header.stamp = rospy.Time().now()
		goal.pose.position.x = 0.0
		goal.pose.position.y = 0.0
		goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z = quaternion_from_euler(0.0, 0.0, 0.0)
		self.nav_to_pose(goal)

	def ard_sub_callback(self, msg):
		flame_threshold = 950.0
		if msg.angular.z < flame_threshold:
			self.flame_seen = True
			self.end_nav()
			self.flame_pub.publish(True) # Stop motors
			self.fan_toggle()
			thing = self.get_candle_xy()
			rospy.loginfo("FOUND CANDLE at [{2}, {3}, th {4} ] and pos [{0},{1}]".format(thing[0], thing[1], self.odom_x, self.odom_y, self.z, self.last_laser_dist))
			timer.sleep(2) # Wait to go home
			self.flame_pub.publish(False) # Stop motors
			self.go_home()
			rospy.loginfo("Going home!")

			# Exit thread
			rospy.signal_shutdown("done")
			exit()

	def scan_sub_callback(self, msg):
		self.last_laser =  msg.ranges
		self.laser_th = msg.angle_increment

	def fan_toggle(self):
		self.fan_pub.publish(True)
		timer.sleep(3)
		self.fan_pub.publish(False)

	def approx(self, cell):
		# approximate a grid cell's location by returning a list
		# of nearby nodes
		unexplored = []
		explored = [cell]

		unexplored.extend(self.get_neighbors(cell))

		while True:
			e = unexplored.pop(0)
			if e in explored:
				continue
			explored.append(e)

			if e.is_empty() and not e.is_unknown():
				return e
			else:
				unexplored.extend(self.get_neighbors(e))

	def check_frontier(self, cell):
		# check if a grid cell lies on a frontier
		return cell.is_unknown() and self.find_near_known(filter(lambda c: c.is_empty(), self.get_neighbors(cell)))


	def find_frontier(self):
		# find a frontier exploration point based on
		# the length and distance away each frontier list is.
		# this will set a new nav goal if the current is unreachable
		frontier = []

		for i in self.cost_grid:
			for j in i:
				if self.check_frontier(j):
					frontier.append(j)
		list_fronts = self.get_grouped(frontier)

		list_fronts = filter(lambda l: self.farthest(l) >= 10, list_fronts)

		for f in list_fronts:
			for c in f:
				self.publish_cell(c.get_x_pos(), c.get_y_pos(), 'front')
		for j in range(10):
			self.publish_cells()

		cents = map(self.get_centroid, list_fronts)
		lens = map(len, list_fronts)
		dists = map(self.dist_to_centroid, cents)
		weights = []

		if len(cents) == 0:
			self.end_nav()
			self.done = True
			print "Finished Mapping"
			# self.exp_cells = []
			exit()

		for c in cents:
			self.publish_cell(c.get_x_pos(), c.get_y_pos(), 'path')
		self.publish_cells()

		try:
			for j in range(len(dists)):
				rospy.loginfo("Lens: {0}".format(lens[j]))

				if dists[j] < 100:
					weights.append(dists[j] / lens[j])
				else:
					weights.append(0)
		except:
			pass

		best = 0
		v = 0
		for i in range(len(weights)):
			if weights[i] > best:
				best = weights[i]
				v = i

		if not self.unreachable:
			prev_goal = self.new_goal
			self.new_goal = cents[v]
			if prev_goal != self.new_goal and self.new_goal != None:
				self.end_nav()
		else:
			self.new_goal = cents[random.randint(0, len(cents)-1)]
			self.unreachable = False

		self.publish_cell(self.new_goal.get_x_pos(), self.new_goal.get_y_pos(), 'exp')
		self.publish_cell(self.new_goal.get_x_pos()+ 1, self.new_goal.get_y_pos(), 'exp')
		self.publish_cell(self.new_goal.get_x_pos()- 1, self.new_goal.get_y_pos(), 'exp')
		self.publish_cell(self.new_goal.get_x_pos(), self.new_goal.get_y_pos()+ 1, 'exp')
		self.publish_cell(self.new_goal.get_x_pos(), self.new_goal.get_y_pos()- 1, 'exp')
		for i in range(10):
			self.publish_cells()

	def get_neighbors(self, cell):
		# return a list of neighbors for a given grid cells
		neighbors = []

		x_pos = cell.get_x_pos()
		y_pos = cell.get_y_pos()

		for row in range(y_pos - 1, y_pos + 2):
			for col in range(x_pos - 1, x_pos + 2):
				if row < 0 or col < 0 or (row == y_pos and col == x_pos):
					continue
				else:
					try:
						neighbors.append(self.cost_grid[col][row])
					except:
						pass

		return neighbors

	def dist_to_centroid(self, cent):
		# return the distance between centroid cells
		cell_x, cell_y = self.map_to_grid(self.x, self.y)
   		return math.sqrt((cent.get_x_pos() - cell_x) ** 2 + (cent.get_y_pos() - cell_y) ** 2)

	def find_near_known(self, neighs):
		# find known cells for a list of neighbors
		for c in neighs:
			if not c.is_unknown():
				return True

		return False

	def is_adjacent(self, a, b):
		# check adjacency
		return (abs(a.get_x_pos() - b.get_x_pos()) <= 1) and (abs(a.get_y_pos() - b.get_y_pos()) <= 1)

	def get_grouped(self, unsort):
		# find the grouped list of frontiers
		frontiers = []

		while len(unsort) != 0:
			frontier = [unsort.pop(0)]
			self.done = False
			while not self.done:
				# try to append adjacent cells,
				# ask for forgiveness to exit loop (raise exception)
				try:
					for a in unsort:
						for b in frontier:
							if self.is_adjacent(a, b):
								frontier.append(a)
								unsort.remove(a)
								raise StopIteration
					self.done = True
					frontiers.append(frontier)
				except:
					pass
		return frontiers

	def update_pose(self, data):
		# update the robot's pose
		try:
			self.odom_x = data.pose.pose.position.x
			self.odom_y = data.pose.pose.position.y
			self.odom_th = data.pose.pose.orientation.z

			pos, ang = self.odom_list.lookupTransform('map', 'base_link', rospy.Time(0))
			self.x = pos[0]
			self.y = pos[1]
			self.xc, self.yc  = self.map_to_grid(pos[0], pos[1])

			rol, pit, yaw = euler_from_quaternion(ang)
			self.z = yaw
		except Exception as e:
			rospy.logwarn("Odom update failed! {}".format(str(e)))
			pass

	def farthest(self, front):
		# return the farthest frontier from the robot based on
		# man distance
		best = 0

		for f in front:
			for b in front:
				dist = math.sqrt((f.get_x_pos() - b.get_x_pos())**2 + (f.get_x_pos() - b.get_x_pos())**2)
				if dist > best:
					best = dist
		return best

	def request_map(self, e):
		# map service request
		get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)
		self.read_map(get_map_srv().map)


	def read_map(self, data):
		# update map data
		self.map_w = data.info.width
		self.map_h = data.info.height
		self.map_data = data.data
		self.map_res = data.info.resolution
		self.map_ox = data.info.origin.position.x
		self.map_oy = data.info.origin.position.y

		# Cost grid is a list of lists of cost grids (2D organization)
		# used to store occupancy and distance information about gridcells
		self.cost_grid = [[0 for x in range(self.map_h)] for x in range(self.map_w)]

		c = 0

		for y in range(0, self.map_h ):
			for x in range(0, self.map_w):
				if c < len(self.map_data):
					self.cost_grid[x][y] = CostCell(x, y, self.map_data[c])
					c += 1

		self.expand_walls()
		self.publish_walls()
		self.find_frontier()

	def expand_walls(self):
		# expand by a radius of 5
		for y in range(0, self.map_h):  # Rows
			for x in range(0, self.map_w):  # Columns
				if self.cost_grid[x][y].get_val() > 90:
					for y2 in range (y - 5, y + 6):
						for x2 in range (x - 5, x + 6):
							if math.sqrt((y2 - y)**2 + (x2 - x)**2) > 5:
								continue
							try:
								self.cost_grid[x2][y2].set_val(60)
							except IndexError:
								pass

	def map_to_grid(self, mx, my):
		# convert map to grid cell coordinates
		return int(math.floor((mx - self.map_ox) / self.map_res)), int(math.floor((my - self.map_oy) / self.map_res))

	def grid_to_map(self, mx, my):
		# convert grid cell to map coor
		return ((mx * self.map_res + self.map_ox) + (self.map_res / 2)), ((my * self.map_res + self.map_oy) + (self.map_res / 2))

	def publish_cell(self, x, y, karg):
		# add cells to their given lists
		# using karg as a multiplexer
		pt = Point()
		pt.z = 0
		pt.x, pt.y = self.grid_to_map(x, y)

		if karg == 'wall':
			self.wall_cells.append(pt)
		elif karg == 'exp':
			self.exp_cells.append(pt)
		elif karg == 'path':
			self.path_cells.append(pt)
		elif karg == 'front':
			self.front_cells.append(pt)

	def publish_cells(self):
		# publish all cells
		self.publish_expanded()
		self.publish_walls()
		self.publish_path()
		self.publish_frontier()

	def publish_expanded(self):
		# publish expanded topic
		cs = GridCells()

		cs.header.frame_id = 'map'
		cs.cell_width = self.map_res
		cs.cell_height = self.map_res
		cs.cells = self.exp_cells

		self.exp_pub.publish(cs)

	def publish_path(self):
		# publish path topic
		cs = GridCells()

		cs.header.frame_id = 'map'
		cs.cell_width = self.map_res
		cs.cell_height = self.map_res
		cs.cells = self.path_cells

		self.path_pub.publish(cs)

	def publish_frontier(self):
		# pubish frontier topic
		cs = GridCells()

		cs.header.frame_id = 'map'
		cs.cell_width = self.map_res
		cs.cell_height = self.map_res
		cs.cells = self.front_cells

		self.front_pub.publish(cs)

	def publish_walls(self):
		# publish walls topic
		self.wall_cells = []
		for y in range(0, self.map_h):
			for x in range(0, self.map_w):
				if not self.cost_grid[x][y].is_empty():
					self.publish_cell(x, y, 'wall')
		cs = GridCells()

		cs.header.frame_id = 'map'
		cs.cell_width = self.map_res
		cs.cell_height = self.map_res
		cs.cells = self.wall_cells

		self.wall_pub.publish(cs)

	def nav_to_pose(self, goalp):
		# nav to pose using move base
		p = MoveBaseGoal()
		p.target_pose.header.frame_id = 'map'
		p.target_pose.header.stamp = rospy.Time.now()
		p.target_pose.pose = goalp.pose
		self.move.send_goal(p)

	def end_nav(self):
		# end the navigation
		c = GoalID()
		self.cancel.publish(c)

	def update_status(self, data):
		# update the goal status
		for goal in data.status_list:
			if goal.status < 2:
				self.last_goal = goal
				self.at_goal = False
		if len(data.status_list) > 0 and not self.at_goal:
			for goal in data.status_list:
				# get next goal, at current goal
				if goal.goal_id.id == self.last_goal.goal_id.id:
					if 2 <= goal.status <= 3:
						self.at_goal = True
						rospy.loginfo("Turning 360!")

						# self.turn360(6)

						self.nav_next_centroid()
					elif 4 <= goal.status <= 5:  # Goal unreachable or rejected
						self.at_goal = True
						self.unreachable = True
						rospy.loginfo("Turning 360 at UNREACHABLE!")
						# self.turn360(6)

						self.end_nav()
						self.nav_next_centroid()

	def nav_next_centroid(self):
		# call nav to pose for the next point
		goal = PoseStamped()
		goal.header.frame_id = 'map'
		goal.header.stamp = rospy.Time().now()
		world_x, world_y = self.grid_to_map(self.new_goal.get_x_pos(), self.new_goal.get_y_pos())
		goal.pose.position.x = world_x
		goal.pose.position.y = world_y
		goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z = quaternion_from_euler(0.0, 0.0, 0.0)
		self.nav_to_pose(goal)

def main():

	rospy.init_node('explorer')

	exp = Explorer()

	rospy.Subscriber('/odom', Odometry, exp.update_pose)
	rospy.Subscriber('/navgoal', PoseStamped, exp.nav_to_pose)
	exp.odom_list = tf.TransformListener()
	exp.move = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	exp.move.wait_for_server(rospy.Duration(5))
	# Subscribe to move base status.
	status = rospy.Subscriber('/move_base/status', GoalStatusArray, exp.update_status)

	exp.request_map(None)

	rospy.sleep(rospy.Duration(5))
	exp.nav_next_centroid()
	rospy.Timer(rospy.Duration(5), exp.request_map)

	rospy.spin()

if __name__ == '__main__':
	main()
