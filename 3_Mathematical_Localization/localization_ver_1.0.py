import csv
import math
import pyproj
import numpy as np
from matplotlib import pyplot as plt



class Localization():
	def __init__(self, file_path):
		self.file_path = file_path
		self.main()
	

	def main(self):
		print("[start ] Localization process")
		self.val_init()				# value init
		self.file_open()			# .csv file open
		self.val_set()				# value set
		self.gps_loading_for_plot()	# gps data loading
		self.KF()					# start estimate pose using KF
		self.result_plot()			# visualization using matplotlib
		self.file_close()			# .csv file close
		print("[finish] Localization process")

	def val_init(self):
		print("[start ] value init")
		self.theta = []
		self.v = []
		self.w = []
		self.result_theta = []
		self.T = 0.01 #100000
		self.result_x = []
		self.result_y = []
		self.gps_x = []
		self.gps_y = []
		self.odom_x = []
		self.odom_y = []
		self.gps_tm_x = []
		self.gps_tm_y = []


	def file_open(self):
		print("[start ] .csv file open")
		self.gps_file = open(self.file_path['gps'], 'r', encoding='utf-8')
		self.imu_file = open(self.file_path['imu'], 'r', encoding='utf-8')
		self.odom_file = open(self.file_path['odom'], 'r', encoding='utf-8')
		self.udp_file = open(self.file_path['udp'], 'r', encoding='utf-8')
		self.gps_data = csv.reader(self.gps_file)
		self.imu_data = csv.reader(self.imu_file)
		self.odom_data = csv.reader(self.odom_file)
		self.udp = csv.reader(self.udp_file)


	def val_set(self):
		print("[start ] value set")
		for idx, udp in enumerate(self.udp):
				if idx == 0:
					pass			
				else:
					Theta = 0.0
					W = 0.0
					V = 0.0			
					Theta = float(udp[3])#/(180*math.pi)		# deg -> rad
					self.theta.append(Theta)

		for idx, odom in enumerate(self.odom_data):
				if idx == 0:
					pass			
				else:
					odom_x = float(odom[5])
					odom_y = float(odom[6])
					self.odom_x.append(odom_x)
					self.odom_y.append(odom_y)

		for idx, imu in enumerate(self.imu_data):
				if idx == 0:
					pass			
				else:
					w = float(imu[24]) #/(180*math.pi)		# deg -> rad
					self.w.append(w)
					v = float(imu[25]) #* 3.6				# m/s -> km/h
					self.v.append(v)


	def calc_odometry(self,x,y,theta,v,w):
		x = x - (v/w)*math.sin(theta) + (v/w)*math.sin(theta + w*self.T)
		y = y + (v/w)*math.cos(theta) - (v/w)*math.cos(theta + w*self.T)
		theta = theta + w*self.T
		v = v
		w = w
		return x,y,theta

	def KF(self):
		print("[start ] estimate pose using KF")
		for i in range(len(self.v)):
			if i == 0:
				theta_old = self.theta[0]
#				x_old = self.odom_x[0] #gps tm 좌표 변환한거 넣어야함
#				y_old = self.odom_y[0]
				x_old = self.gps_tm_x[0] #gps tm 좌표 변환한거 넣어야함
				y_old = self.gps_tm_y[0]


			x_new, y_new, theta_new = self.calc_odometry(x_old,y_old,theta_old,self.v[i],self.w[i])
			self.result_x.append(x_new)
			self.result_y.append(y_new)
			self.result_theta.append(theta_new)
			x_old = x_new
			y_old = y_new
			theta_old = theta_new

	def gps_loading_for_plot(self):
		for idx, data in enumerate(self.gps_data):
			if idx == 0 :
				pass			
			else:
				gps_x = float(data[7])
				gps_y = float(data[8])
				self.gps_x.append(gps_x)
				self.gps_y.append(gps_y)
		self.change_tm()		

	def change_tm(self):
		LATLONG_WGS84 = pyproj.Proj("+proj=latlong +datum=WGS84 +ellps=WGS84")
		TM127 = pyproj.Proj("+proj=tmerc +lat_0=38N +lon_0=127E +ellps=bessel +x_0=200000 +y_0=600000 +k=1.0 ")
		for i in range(len(self.gps_x)):
			longt = float(self.gps_x[i])
			latti = float(self.gps_y[i])
			tm_x, tm_y = pyproj.transform(LATLONG_WGS84, TM127, latti, longt)
			self.gps_tm_x.append(tm_x)
			self.gps_tm_y.append(tm_y)

	def result_plot(self):
		plt.subplot(2,2,1)
		plt.title('Estimated')
		plt.plot(self.result_x, self.result_y)
		plt.subplot(2,2,2)
		plt.title('GNSS(WGS84)')
		plt.plot(self.gps_x, self.gps_y, 'r')
		plt.subplot(2,2,3)
		plt.title('GNSS(TM)')
		plt.plot(self.gps_tm_x, self.gps_tm_y, 'g')
		plt.subplot(2,2,4)
		plt.title('Estimated + GNSS(TM)')
		plt.plot(self.result_x, self.result_y, 'b', label='Estimated')
		plt.plot(self.gps_tm_x, self.gps_tm_y, 'g', label='GNSS(TM)')
		plt.legend()
		plt.show()
		

	def file_close(self):
		print("[start ] .csv file close")
		self.gps_file.close()
		self.imu_file.close()
		self.odom_file.close()
		self.udp_file.close()



def main():
#	file_path = {'gps':'./csv_7/gps.csv', 'imu':'./csv_7/imu.csv', 'odom':'./csv_7/odom.csv', 'udp':'./csv_7/udp.csv'} # T = 0.01
	file_path = {'gps':'./csv_5/gps.csv', 'imu':'./csv_5/imu.csv', 'odom':'./csv_5/odom.csv', 'udp':'./csv_5/udp.csv'} # T = 0.01
#	file_path = {'gps':'./csv_4/gps.csv', 'imu':'./csv_4/imu.csv', 'odom':'./csv_4/odom.csv', 'udp':'./csv_4/udp.csv'} # T = 0.1
	localization = Localization(file_path)

if __name__ == "__main__":
	main()







