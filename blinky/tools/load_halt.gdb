target remote :3333

define reset
  monitor reset halt
end

monitor reset halt
load
monitor reset halt
