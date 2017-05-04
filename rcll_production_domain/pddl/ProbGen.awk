BEGIN { 
	FS = "\t"
	f = 1
	products = 0
	ring_colors[1] = "blue"
	ring_colors[2] = "green"
	ring_colors[3] = "yellow"
	ring_colors[4] = "orange"
}

#	Check for initialization of new problem
/NEWP/{
	if (flag == 0) {
		print "Reading file", FILENAME 
		flag = 1
	}	
	file = "ProblemSet/prob" f ".pddl"
	print file
	print "New Problem -- Products = "$2
	products = $2
#	for (e in attrib) {
#		delete attrib[e]
#	}
	delete attrib
	for (e in ring_attrib) {
		delete ring_attrib[e]
	}
	for (e in cap) {
		delete cap[e]
	}
	
	
	i = 1
} 
#	Check for end of problem, generate problem file
/ENDP/ {

#	Problem File definition starts
	print "(define (problem p"f")" > file
	print "(:domain rcll-production-steps)" > file
	print "(:objects" > file
	print "\t; robots" > file
	print "\tr1 r2 r3 - robot" > file
	print "" > file
	print "\t; products" > file
	for (ct = 1; ct <= products; ct++) {
		print "\tp" ct " - product" > file
		step = "\tinit_p" ct
		if (attrib[ct ",ring1"] != "-") {
			step = step " ring1_p" ct
		}
		if (attrib[ct ",ring2"] != "-") {
			step = step " ring2_p" ct
		}
		step = step" cap_p" ct " delivery_p" ct " - step"
		print step > file
	}
	print ")" > file
	print "" > file

#	Robot Initialization
	print "(:init\n;robots\n\t(robot-at-init r1)\n\t(robot-at-init r2)\n\t(robot-at-init r3)\n" > file

#	Initial Product Propositions
	for (ct = 1; ct <= products; ct++) {
		print ";p" ct > file
#		Base Steps
		print "\t(initial-step init_p" ct ")" > file
		print "\t(has-step p" ct " init_p" ct ")" > file
		print "\t(step-at-machine init_p" ct " bs)" > file
#		Ring Steps
		if (attrib[ct ",ring1"] != "-") {
			print "\t(step-precedes init_p" ct " ring1_p" ct ")" > file
			print "\t(has-step p" ct " ring1_p" ct ")" > file
			print "\t(step-at-machine ring1_p" ct " " ring_attrib[attrib[ct ",ring1"] ",station"] ")" > file
			print "\t(= (material-required ring1_p" ct ") " ring_attrib[attrib[ct ",ring1"] ",requirement"] ")" > file
#			print attrib[ct ",ring1"]
		}
		if (attrib[ct ",ring2"] != "-") {
			print "\t(step-precedes ring1_p" ct " ring2_p" ct ")" > file
			print "\t(has-step p" ct " ring2_p" ct ")" > file
			print "\t(step-at-machine ring2_p" ct " " ring_attrib[attrib[ct ",ring2"] ",station"] ")" > file
			print "\t(= (material-required ring2_p" ct ") " ring_attrib[attrib[ct ",ring2"] ",requirement"] ")" > file
			print attrib[ct ",ring2"]
		}
#		Cap Steps
		if (attrib[ct ",ring1"] == "-") 
			print "\t(step-precedes init_p" ct " cap_p" ct ")" > file
		else if (attrib[ct ",ring2"] == "-") 
			print "\t(step-precedes ring1_p" ct " cap_p" ct ")" > file
		else
			print "\t(step-precedes ring2_p" ct " cap_p" ct ")" > file
		print "\t(has-step p" ct " cap_p" ct ")" > file
		print "\t(step-at-machine cap_p" ct " " cap[attrib[ct ",cap"] ",station"] ")" > file
#		Delivery Steps
		print "\t(step-precedes cap_p" ct " delivery_p" ct ")" > file
		print "\t(has-step p" ct " delivery_p" ct ")" > file
		print "\t(step-at-machine delivery_p" ct " ds)" > file
	}

#	Initial Machine Propositions
	print "; stations\n\t(= (material-load rs1) 0)\n\t(= (material-load rs2) 0)\n\t(output-location bs_out bs)" > file
	print "\t(input-location rs1_in rs1)\n\t(output-location rs1_out rs1)\n\t(input-location rs2_in rs2)\n\t(output-location rs2_out rs2)" > file
	print "\t(input-location cs1_in cs1)\n\t(output-location cs1_out cs1)\n\t(input-location cs2_in cs2)\n\t(output-location cs2_out cs2)\n\t(input-location ds_in ds)\n)\n" > file

#	Goal Propositions
	print "(:goal\n\t(and" > file
	for (ct = 1; ct <= products; ct++) {
		print "\t\t(step-completed delivery_p" ct ")" > file
	}
	print "\t\t(forall (?l - location) (not (location-full ?l)))\n\t\t(= (material-load rs1) 0)\n\t\t(= (material-load rs2) 0)\n\t)\n)\n" > file

#	Metric
	print "(:metric minimize (total-time))\n)" > file
	f++
}

	#Read and store product information
/prod/ {
	attrib[i ",base"] = $2
	attrib[i ",ring1"] = $3
	attrib[i ",ring2"] = $4
	attrib[i ",cap"] = $5
	print "i", i
#	print "base", attrib[i",base"]
#	print "ring1", attrib[i",ring1"]
#	print "ring2", attrib[i",ring2"]
#	print "cap", attrib[i",cap"]
	for (e in attrib)
		print e"\t"attrib[e]
	print ""
	i++
}
	
/rings/ {
	print "Found ring color specification"
	for (col in ring_colors) {
#		print "Ring Color", ring_colors[col]
		if (ring_colors[col] == $2) {
			ring_attrib[ring_colors[col]",station"] = $4
			ring_attrib[ring_colors[col]",requirement"] = 1
#			print "Ring Station", ring_attrib[ring_colors[col]",station"], "Material Requirements", ring_attrib[ring_colors[col]",requirement"]
		} else if (ring_colors[col] == $3) {
			ring_attrib[ring_colors[col]",station"] = $4
			ring_attrib[ring_colors[col]",requirement"] = 2
#			print "Ring Station", ring_attrib[ring_colors[col]",station"], "Material Requirements", ring_attrib[ring_colors[col]",requirement"]
		}
	}
	for (e in ring_attrib)
		print e"\t"ring_attrib[e]
}
	
/caps/ {
	cap[$2",station"] = $3
	for (e in cap)
		print e"\t"cap[e]
}

END {

}
