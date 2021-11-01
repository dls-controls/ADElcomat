from iocbuilder import AutoSubstitution, Device
from iocbuilder.arginfo import *
from iocbuilder.modules.ADCore import ADCore
from iocbuilder.modules.asyn import AsynPort

# Template class that is parsed automatically
class _ADElcomatTemplate(AutoSubstitution):
    TemplateFile = 'ADElcomat.template'

# Main device class
class ADElcomat(AsynPort):

    # Dependencies
    Dependencies = (ADCore,)

    # Database definitions
    DbdFileList = ['ADElcomat']

    # Library
    LibFileList = ['ADElcomat']

    # Is an Asyn device
    IsAsyn = True

    def __init__(self, name, P, R, SPORT, SADDR=0, BUFFERS=50, MEMORY=0,
            PRIORITY=0, STACKSIZE=0, TIMEOUT=1, **args):
        self.__super.__init__(name)
        self.__dict__.update(locals())

        # The template wants the asyn port of this driver
        _ADElcomatTemplate(PORT=name, ADDR=0, P=P, R=R, TIMEOUT=TIMEOUT)

    def InitialiseOnce(self):
        print("# ADElcomatConfig(portName, serialPortName, serialPortAddress)")

    def Initialise(self):
        print(
            "ADElcomatConfig(\"{name}\", \"{serial_port}\", {serial_address}, {buffers}, {memory}, {priority}, {stacksize})".format(
                name=self.name,
                serial_port=self.SPORT,
                serial_address=self.SADDR,
                buffers=self.BUFFERS,
                memory=self.MEMORY,
                priority=self.PRIORITY,
                stacksize=self.STACKSIZE))

    ArgInfo = makeArgInfo(__init__,
        name = Simple("Object and asyn port name", str),
        P = Simple("PV prefix 1", str),
        R = Simple("PV prefix 2", str),
        SPORT = Simple("Serial Port's Port", str),
        SADDR = Simple("Serial Port's Addr", int),
        BUFFERS = Simple("Number of buffers", int),
        MEMORY = Simple("Memory", int),
        PRIORITY = Simple("Priority", int),
        STACKSIZE = Simple("Stack size", int),
        TIMEOUT = Simple("Timout", int)
        )
