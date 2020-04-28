rb = 100

start_d = 1.0
robot_d=0.5;
d_lr=0.25;
d_fb=0.25;
lr_step=robot_d+d_lr
fb_step=robot_d+d_fb
limit_d = -start_d
x = Array((limit_d..start_d).step(lr_step))

level = []
id = 0
s = 0
require 'set'
r_positions = Set.new
#puts x.join(",")
while id <= rb
    s = s + 1
    #puts x.join(",")
	#puts [start_d,limit_d].join(",")
    for p in [start_d,limit_d]
    	for p1 in x
			pp = []
    		if s % 2 == 0
    			puts [id,s,p1,p].join(",")
    			pp = [p1,p]
    		else
				puts [id,s,p,p1].join(",")
				pp = [p,p1]
    		end
    		if r_positions.add?(pp) != nil
    		
				id = id + 1
			end
    		if id >rb
				break
			end
    	end
    	if id >rb
			break
		end
    end
  #  puts level.join(", ")
    if s % 2 == 0
		puts "next level"
    	start_d = start_d + fb_step
    	limit_d = -start_d
    	x = Array((limit_d..start_d).step(lr_step))
    	
    end
end
