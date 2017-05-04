BEGIN {
	file = "log.txt"
	ring_colors[1] = "blue"
	ring_colors[2] = "green"
	ring_colors[3] = "yellow"
	ring_colors[4] = "orange"
	col_string = ""
	cap_string = ""
}

/NUMP/ {
	NUMP = $2
}

/MAX/ {
	MAX_PROD = $2
}
	
/rs/ {
#	print "Found ring color specification" > file
#	ring_attrib[$2 ",station"] = $1
#	ring_attrib[$2 ",requirement"] = 1
#	ring_attrib[$3 ",station"] = $1
#	ring_attrib[$3 ",requirement"] = 2

	col_string = col_string $0 "\n"

#	for (e in ring_attrib)
#		print e"\t"ring_attrib[e] > file
}
	
/caps/ {
#	cap[$2",station"] = $3
#	for (e in cap)
#		print e"\t"cap[e] > file

	cap_string = cap_string $0 "\n"
}

END {
	for (prob = 1; prob <= NUMP; prob++) {
		nprod = int(rand()*MAX_PROD)
		nprod = nprod == 0 ? 1 : nprod
		print "NEWP\t" nprod
#	Fixed (for now) Information about the environment
		printf "%s", col_string
		printf "%s", cap_string
		for (i = 1; i <= nprod; i++) {
			delete ring
			out = "prod" i
			x = rand()
			base = x < 0.33 ? "black" : x < 0.67 ? "white" : "red"
			out = out "\t" base
			nrings = int(rand()*3)
			nrings = nrings == 3 ? 2 : nrings
			for (ct = 1; ct <= 2; ct++) {
				if(ct <= nrings) {
					x = rand()			
					ring[ct] = x < 0.25 ? "blue" : x < 0.5 ? "green" : x < 0.75 ? "yellow" : "orange"
					out = out "\t" ring[ct]
				} else {
					out = out "\t-"
				}
			}
			cap = rand() < 0.5 ? "black" : "white"
			out = out "\t" cap
			print out
		}
		print "ENDPROBLEM"
	}
}
