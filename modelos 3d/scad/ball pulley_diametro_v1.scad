
PI = 3.1415927;

tolerancia=0.015;
detalle=20;
diametro_ideal=50;

ball_diameter = tolerancia+ 4.2;
//cantidad = 96; // for regular pully;wd
link_largo =   tolerancia+  1.8;
espaciado = ball_diameter+link_largo; 

cantidad= round(diametro_ideal/espaciado*PI);
//echo(cantidad);

//link_diameter = 0.9;
link_diameter =  tolerancia+  1.28;

diametro =  cantidad*espaciado / PI;
altura =   ball_diameter*3.5;

altura_agarre = altura ;

module pulley () {
 difference(){
	difference()
	{
		cylinder (altura_agarre, r = diametro/2, center=true, $fn = cantidad);

		for (i = [1:cantidad])
		{	
			rotate ([0, 0, (360/cantidad) * i])		
			translate ([diametro/2,0,0])
			sphere (r = ball_diameter/2,$fn=detalle);
     
		}

	}
  	
		rotate ([0, 0, -90]) // Rotate -90 to match face direction of cylinder
		rotate_extrude (convexity = 5, $fn = cantidad)
		translate ([diametro/2, 0, 0])
		circle (r = link_diameter/2, center = true);
 }
}

//translate ([0, 0, altura_polea/2])
pulley(); 
