
tolerancia=0.015;
offset_whell=0;
detalle=20;
ball_diameter = tolerancia+ 4.2;
ball_count = 96; // for regular pully;wd
link_largo =   tolerancia+  1.8;
// ball_spacing = 5.1; // The distance from the centre of one ball to the next
ball_spacing = ball_diameter+link_largo; // The distance from the centre of one ball to the next
//link_diameter = 0.9;
link_diameter =  tolerancia+  1.28;

PI = 3.1415927;
wheel_diameter =  tolerancia+  ball_count*ball_spacing / PI;
wheel_height =   ball_diameter*3.5;

pulley_height = wheel_height ;

module pulley () {
 difference(){
	difference()
	{
		cylinder (pulley_height, r = wheel_diameter/2+offset_whell, center=true, $fn = ball_count);

		for (i = [1:ball_count])
		{	
			rotate ([0, 0, (360/ball_count) * i])		
			translate ([wheel_diameter/2,0,0])
			sphere (r = ball_diameter/2,$fn=detalle);
     
		}

	}
  	
		rotate ([0, 0, -90]) // Rotate -90 to match face direction of cylinder
		rotate_extrude (convexity = 5, $fn = ball_count)
		translate ([wheel_diameter/2, 0, 0])
		circle (r = link_diameter/2, center = true);
 }
}

//translate ([0, 0, pulley_height/2])
pulley(); 
