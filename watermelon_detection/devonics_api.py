# Slave Address - Function - Register Address - Register Data - CRC
from dobot_api import parseResponse
import time  
class DevonicsApi:
    dataType = "U16"

    def __init__(self, dashboard, ip, port, slave_id, rtu):
        self.dashboard = dashboard
        self.ip = ip
        self.port = port
        self.slave_id = slave_id
        self.rtu = rtu
        self.index = 0

        self.commands = {
            "INIT_STAT": 0,
            "REQ_POS": 1,
            "ACT_POS": 2,
            "MRPM": 3
            
        }

        self.dashboard.ModbusCreate(self.ip,self.port,self.slave_id,self.rtu)

    def disconnect(self):
        self.dashboard.ModbusClose(self.index)

    def printIndex(self):
        print(self.index)

    def printCommands(self):
        for command in self.commands:
            print(self.commands[command])

    # requires a move.Sync() after to block until initialized
    # Can perform Initialization or Full initialization
    def initialize(self, reinitialize=1):
        count = 1
        self.dashboard.SetHoldRegs(self.index,self.commands["INIT_STAT"], count,0, self.dataType)
  
    def getInitializationState(self):
        count = 1 
        data = self.dashboard.GetHoldRegs(self.index, self.commands["initializationState"], count, self.dataType)
        return data
    
    def setPosition(self,position):
        count = 1
        self.dashboard.SetHoldRegs(self.index,self.commands["REQ_POS"], count, position, self.dataType)     

    def getPosition(self):
        count = 1
        data = self.dashboard.GetHoldRegs(self.index,self.commands["ACT_POS"],count,self.dataType)
        parsed_data = parseResponse(data)
        if parsed_data == "" or parsed_data is None:
            return -1
        return parseResponse(data)
    
    def setSpeed(self,speed):
        count = 1
        self.dashboard.SetHoldRegs(self.index, self.commands["MRPM"], count,speed,self.dataType)

    def getSpeed(self):
        count = 1
        data =self.dashboard.GetHoldRegs(self.index, self.commands["MRPM"], count, self.dataType)
        parsed_data = parseResponse(data)
        if parsed_data == "" or parsed_data is None:
            return -1
        return parseResponse(data)

