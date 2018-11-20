import serial

from tkinter import Tk
from tkinter import Label
from tkinter import Button
from tkinter import Scale
from tkinter import HORIZONTAL


BAUD = 57600
PORT = '/dev/ttyUSB0'

DEFAULT_LENGTH = 1600

DEFAULT_SETPOINT = 3.0
DEFAULT_P = 100
DEFAULT_I = 50
DEFAULT_D = 35

serial_value = ''


# def process_serial():
#     ser = serial.Serial(PORT, BAUD)
#     while True:
#         for x in serial_value:



class MyFirstGUI:
    def __init__(self, master):
        self.serial = serial.Serial(PORT, BAUD)

        self.master = master
        master.title("Interface to SB Robot")

        self.label = Label(master, text="Control Interface")
        self.label.pack()

        self.close_button = Button(master, text="Close", command=master.quit)
        self.close_button.pack()

        self.setpoint = Scale(master, from_=1, to=5, resolution=0.05,
                              orient=HORIZONTAL, command=self.setpoint_update,
                              length=DEFAULT_LENGTH,
                             )
        self.setpoint.set(DEFAULT_SETPOINT)
        self.setpoint.pack()

        self.p = Scale(master, from_=0, to=1000,
                       orient=HORIZONTAL, command=self.p_update,
                       length=DEFAULT_LENGTH,
                      )
        self.p.set(DEFAULT_P)
        self.p.pack()

        self.i = Scale(master, from_=0, to=1000,
                       orient=HORIZONTAL, command=self.i_update,
                       length=DEFAULT_LENGTH,
                      )
        self.i.set(DEFAULT_I)
        self.i.pack()

        self.d = Scale(master, from_=0, to=1000,
                       orient=HORIZONTAL, command=self.d_update,
                       length=DEFAULT_LENGTH,
                      )
        self.d.set(DEFAULT_D)
        self.d.pack()

    def setpoint_update(self, value):
        self.send_update('S', value)

    def p_update(self, value):
        self.send_update('P', value)

    def i_update(self, value):
        self.send_update('I', value)

    def d_update(self, value):
        self.send_update('D', value)

    def send_update(self, header, value):
        s = "{}{:.2f}$".format(header, float(value))
        self.serial.write(s)
        print s
        print self.serial.read_all()

root = Tk()
my_gui = MyFirstGUI(root)
root.mainloop()
