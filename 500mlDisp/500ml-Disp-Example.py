import threading
from devonics_api import DevonicsApi
from dobot_api import DobotApi,DobotApiMove,DobotApiDashboard, MyType
from time import sleep
import numpy as np
import schedule

def connectRobot():
    try:
        ip = "192.168.5.1"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004


        print("Connecting...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print("Connection Successful!")
        return dashboard, move, feed
    except Exception as e:
        print("Failed to connect")
        raise e
    
def connectTool(dashboard):
    try:
        ip = "127.0.0.1"
        port = 60000
        slave_id = 1
        rtu = 1
        model = "PGC-50"

        Tool = DevonicsApi(dashboard,ip,port,slave_id,rtu)
        print("Tool Connection Successful!")
        return Tool
    except Exception as e:
        print("Tool Connection Failed")
        raise e

def get_feed(feed: DobotApi):
    global feedBackData
    global current_actual
    global payload
    global centerX
    global centerY
    global centerZ
    hasRead = 0

    data = bytes()
    while hasRead < 1440:
        temp = feed.socket_dobot.recv(1440 - hasRead)
        if len(temp) > 0:
            hasRead += len(temp)
            data += temp

    a = np.frombuffer(data, dtype=MyType)
    if hex((a['test_value'][0])) == '0x123456789abcdef':
        # Refresh Properties
        feedBackData = a
        current_actual = a["tool_vector_actual"][0]
        running_status = a["running_status"][0]
        payload = a["load"][0]
        centerX = a["center_x"][0]
        centerY = a["center_y"][0]
        centerZ = a["center_z"][0]
        # print("Running_Status", running_status)
        # print("Payload:", feedBackData["load"][0])
    sleep(0.001)

def waitTillReachNWater(Tool, CylinderP):
    perPlantWaitTime = 2
    cylinderMoveStep = int(30000//30)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)


def runRobot(dashboard, move, Tool):
    
    print("move to above tank")
    dashboard.SpeedFactor(40)
    move.ServoJ(-51.6872, 8.8592, -95.0020, -91.1797, -109.6097, -178.4989) 
    sleep(6)

    print("discharge")
    Tool.setSpeed(300)
    move.Sync()
    sleep(1)
    Tool.setPosition(56500)
    move.Sync()
    sleep(22)
    
    print("move into tank")
    dashboard.SpeedFactor(20)
    move.ServoP(233.9609, -558.4424, 220.6578, -88.1298, -0.7438, -122.5535) 
    sleep(10)

    print("suck water")
    CylinderP=int(25000)
    Tool.setPosition(CylinderP)
    move.Sync()
    sleep(23)

    print("move to above tank")
    dashboard.SpeedFactor(40)
    move.ServoJ(-51.6872, 8.8592, -95.0020, -91.1797, -109.6097, -178.4989) 
    sleep(4)

    Tool.setSpeed(100)
    move.Sync()
    sleep(1)
    
    perPlantWaitTime = 4
    
    cylinderMoveStep = int(30000//30)

    z = 450.0
    print("move to Unit 1")
    for x in range(4):
        move.ServoP(77.8626, -724.6656-x*90, z, -88.1133, 1.9703, -141.1184)
       # waitTillReachNWater(Tool, CylinderP)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)

    for x in range(3):
        move.ServoP(77.8626-76.2, -724.6656+x*90-90*2.5, z, -88.1133, 1.9703, -141.1184)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)

    for x in range(4):
        move.ServoP(77.8626-150, -724.6656-x*90, z, -88.1133, 1.9703, -141.1184)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)
        
    print("Move to Unit 2")
    for x in range(4):
        move.ServoP(-356.3812, -683.4988-x*90, z, -85.0498, 4.0268, -172.7899)
        if x == 0:
            sleep(5)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)

    for x in range(3):
       
        move.ServoP(-356.3812-76.2, -683.4988+x*90-90*2.5, z, -85.0498, 4.0268, -172.7899)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)

    for x in range(4):
        move.ServoP(-356.3812-150, -683.4988-x*90, z, -85.0498, 4.0268, -172.7899)
        sleep(perPlantWaitTime)
        print("dispense water")
        CylinderP=CylinderP+cylinderMoveStep
        Tool.setPosition(CylinderP)
        sleep(perPlantWaitTime)
   
    '''
    print("move to Unit 1 plant 8")
    move.ServoP(123.512, -949.5872, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(2)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setSpeed(300)
    move.Sync()
    sleep(1)
    Tool.setPosition(CylinderP)
    move.Sync()
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 9")
    move.ServoP(121.5229, -860.0504, 468.3753, -88.1312, -0.7428, -157.511)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 10")
    move.ServoP(126.2917, -764.9484, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 11")
    move.ServoP(77.8626, -724.6656, 352.6821, -88.1133, 1.9703, -141.1184)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 5")
    move.ServoP(54.9550, -720.3689, 468.3753, -88.1312, -0.7428, -157.511)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 6")
    move.ServoP(60.4338, -816.9608, 468.3753, -88.1312, -0.7428, -157.511)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 7")
    move.ServoP(39.2518, -897.5782, 468.3753, -88.1312, -0.7428, -157.511)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 1")
    move.ServoP(-37.0941, -952.9294, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 2")
    move.ServoP(-29.6915, -857.5894, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 3")
    move.ServoP(-17.0541, -772.9894, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 1 plant 4")
    move.ServoP(-21.0520, -672.8494, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    #unit 2
    print("move to Unit 2 plant 8")
    #x=-352.575
    move.ServoP(-371.7520, -921.2393, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(5)
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 9")
    move.ServoP(-354.5320, -863.4593, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)


    print("move to Unit 2 plant 10")
    move.ServoP(-361.0720, -758.2385, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 11")
    move.ServoP(-371.3920, -662.5385, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 5")
    move.ServoP(-446.0320, -703.8785, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 6")
    move.ServoP(-436.7320, -805.6985, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 7")
    move.ServoP(-443.6920, -885.6185, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 1")
    move.ServoP(-507.6519, -934.2785, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 2")
    move.ServoP(-519.6920, -846.7985, 468.3753, -88.1312, -0.7428, -157.511)  
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 3")
    move.ServoP(-509.8120, -744.1198, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)

    print("move to Unit 2 plant 4")
    move.ServoP(-518.2919, -665.2998, 468.3753, -88.1312, -0.7428, -157.511) 
    sleep(perPlantWaitTime)
    print("dispense water")
    CylinderP=CylinderP+cylinderMoveStep
    Tool.setPosition(CylinderP)
    sleep(perPlantWaitTime)'''

    print("move to water tank")
    move.ServoJ(-51.6872, 8.8592, -95.0020, -91.1797, -109.6097, -178.4989)
    sleep(4)

def main():
    dashboard, move, feed = connectRobot()
    Tool = connectTool(dashboard)

    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()

    dashboard.EnableRobot(1.0,0,0,30)

    #runRobot(dashboard, move, Tool)

    schedule.every().day.at("09:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("09:17").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("10:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("10:40").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("11:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("11:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("11:55").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("12:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("12:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("12:40").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("13:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("13:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("13:40").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("14:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("14:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("14:40").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("15:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("15:20").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("15:40").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("16:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("16:30").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("17:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("17:30").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("18:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("18:30").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("19:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("19:30").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("20:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("21:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("22:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("23:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("23:59").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("01:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("02:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("03:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("04:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("05:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("06:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("07:00").do(runRobot,dashboard, move, Tool)
    schedule.every().day.at("08:00").do(runRobot,dashboard, move, Tool)
    while True:
        schedule.run_pending()
        sleep(1)

main()
