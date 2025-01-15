# Master KRAI package


### Pre-requisites:

 **install:**	
 sudo apt-get install python-is-python3

 sudo apt-get install ros-noetic-catkin python-catkin-tools

 sudo apt-get install ros-noetic-rosserial

 sudo apt-get install ros-noetic-joy-teleop ros-noetic-teleop-twist-joy ros-noetic-joy

**note**:
 kalo error install seperti:
 `E: Unable to locate package pyhton-catkin-tools`
 ganti tulisan python ke python3


### scripts:
  - kelinci.py
  - gajah.py 	
### launch:
  - kelinci.launch
  - gajah.launch

 	# di launch ada parameter dalam namespace node kinematik:
 	`<param name="alpha_coeff" type="double" value="0.5"/>`
 	`<param name="vel_coeff" type="double" value="1"/>`

 	ubah value untuk menyesuaikan kec. angular robot (alpha_coeff) dan kec. linear robot (vel_coeff)
 	min val = 0, max val = 1

 	# dalam namespace node rosserial ada param:
 	`<param name="port" value="$(arg stm)"/>`

 	arg stm diambil dari arg global launch, yaitu:
 	`<arg name="stm" default="/dev/ttyACM0"/>  
     <arg name="stm_id" default="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495254707867251444-if02"/>`

     jika mau pake ttyACM0, ubah: `<param name="port" value="$(arg stm)"/>`

     kalau dgn serial id usb stlink:
     `<arg name="stm_id" default="/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066EFF495254707867251444-if02"/>`

     **note:**
     setiap stm32 nucleo atau stlink punya serial id yg beda:
     sesuaikan dgn cara liat di terminal (ganti stm32 ganti id):
     `ls /dev/serial/by-id/`

     # terdapat parameter untuk load config secara global di launch file:
     `<rosparam command="load" file="$(find master_krai)/config/var_gajah.yaml"/>`

     sesuaikan dengan variabel yang digunakan

     # ada baris untuk include file launch package lain spt:
     `<include file="$(find joy_2)/launch/ps4joy.launch"/>`     
     
     yang artinya akan me-launch file dari pkg tersebut

     lalu ada arg launch file pkg lain, yg bisa diatur di launch file pkg lainnya:
     `<arg name="dev" value="/dev/input/js0"/>
      <arg name ="conf" default="$(find joy_2)/config/ps4joy.yaml"/>`

     arg dev artinya adalah arg pkg joy_2 dimana dia mendapatkan input joystick

     arg conf adalah arg pkg joy_2 dimana dia mengatur mapping atau 
     konfigurasi lain sesuai dengan config yang diberikan di arg tsb







## note
 	di semua launch ada config untuk mengubah setting input joystick:
 	di package joy_2 buat file config baru di:
 	`~/ros_ws/src/joy_2/config`

 	sesuaikan dengan input, caranya di terminal ketik:
 	`bluetoothctl`		*koneksikan dulu stick ke bluetooth*
 	`scan on`			*tunggu hingga muncul address stick*
 	`trust [address stick]`
 	`pair [address stick]`
 	`connect [address stick]`

 	cek input joystick:
 	`jstest /dev/input/js0`

 	akan muncul tampilan:
 	`Axes 0:	1:		.... Buttons 0:		1:	` *sambil ditest mana input yang sesuai*

 	kalo tidak ada, cari ada berapa input yang terdeteksi di jetson:
 	`ls /dev/input/js*`

 	di config joy_2, buat config yang sesuai dengan hasil jstest tadi: 	
 	`nano baru.conf` *di ~/ros_ws/src/joy_2/config/*

 	lihat contoh di ps4joy.yaml, ada list:
 	`mappings:
 		- axes:
 		- buttons:`

 	sesuaikan urutan dari hasil jstest, misal:
 	di jstest:
 	`Axes 5: 0` adalah RX, `Axes 3: 0` adalah RY
 	`Buttons  7: 0` adalah kotak `Buttons  1: 0` adalah X
 	di conf  : 	
	`mappings:
 		- axes:
 			- axes[5] #urutan array/list pertama, jadi msg.axes[0]
 			- axes[3] #urutan array/list kedua, jadi msg.axes[1]
 		- buttons:
 			- buttons[7] #urutan array/list pertama, jadi msg.buttons[0]
 			- buttons[1] #urutan array/list pertama, jadi msg.buttons[1]`  	

 	jadi, di script python ketik hasil mapping tsb sesuai config:
 	contoh:
 	`msg.axes[0]: robot kiri kanan`
 	`msg.axes[1]: robot maju mundur`

 	`if msg.buttons[0] == True: kotak on
 	 elif msg.buttons[0] == False: kotak off`

 	 `if msg.buttons[1] == True: X on
 	 elif msg.buttons[1] == False: X off`


## note untuk config joy pake ds4drv

### ada beberpa cara koneksi nya:
 1. pake command di terminal `ds4drv &` *resiko koneksi ke ps4 stick lain*

 2. pake command di terminal `ds4drv --hidraw &` *koneksi ke DEVICE yang SUDAH di trust & pair di bluetoothctl* 
 sehingga mencegah device ps4 lain konek

 3. menggunakan script `ds4_run.py`, tinggal di un-comment di launch file


## Note file variable.yaml
### idle atau pwm 3000 di list pwm bldc (bldc mati) dihapus aja
### di program script sudah ada command untuk matikan bldc (tombol PS)
### jadi tinggal tombol atas bawah  untuk ganti setting pwm(lihat file var_gajah)
### klo mau liat kecepatan tinggal un-comment line print di script python
### tidak perlu buka terminal trus rostopic echo