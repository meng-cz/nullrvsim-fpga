
import sys 

if len(sys.argv) < 4:
    print("Usage: %s examplerocketsystem.v nulctrlmp.v output.v" % sys.argv[0])
    exit(-1)

rocket_path = sys.argv[1]
nulctrl_path = sys.argv[2]
output_path = sys.argv[3]

rocket_lines = []
nulctrl_lines = []

with open(rocket_path, "r") as ifile:
    rocket_lines = ifile.readlines()
with open(nulctrl_path, "r") as ifile:
    nulctrl_lines = ifile.readlines()

nulctrl_module_name = "NulCPUCtrlMPWithUart"
nulctrl_port_prefix = "io_cpu_"
nulctrl_tx_port = "io_txd"
nulctrl_rx_port = "io_rxd"

# find nulctrl_ports

nul_input_ports = []
nul_output_ports = []
nul_cpu_num = 0

l = 0
for l in range(len(nulctrl_lines)):
    if nulctrl_lines[l].startswith("module %s" % nulctrl_module_name):
        l += 1
        break

while not nulctrl_lines[l].startswith(")"):
    words = nulctrl_lines[l].replace(",", "").replace("\n", "").split(" ")
    words = [v for v in words if len(v)]
    name = words[-1]
    if not name.startswith(nulctrl_port_prefix):
        l += 1
        continue
    name = name[len(nulctrl_port_prefix):]
    idx = name.find("_")
    cpu = int(name[:idx])
    port = name[idx+1:]
    if cpu == 0:
        if words[0] == "input":
            nul_input_ports.append(port)
        else:
            nul_output_ports.append(port)
    nul_cpu_num = max(nul_cpu_num, cpu)
    l += 1
nul_cpu_num += 1

print("NulCtrl has %d CPU Ports" % nul_cpu_num)
print("Input Ports: %s" % ", ".join(nul_input_ports))
print("Output Ports: %s" % ", ".join(nul_output_ports))



