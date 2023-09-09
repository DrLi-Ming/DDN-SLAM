$(function(){
	/**
 * @author Nicolas Barradeau 
 * http://en.nicoptere.net
 */
(function() 
{
	var c;
	var ctx;
	var width;
	var height;
	
	var axes = [];
	var wanderers = [];
	var colors = [	
					"#FFCC00", 
					"#66CCF0", 
					"#FF0033", 
					"#99CC33"
				];
	
	window.onload = function()
	{
		
		c = document.getElementById( "canvas" );
    c.width = window.innerWidth
    c.width = window.innerHeight
		ctx = c.getContext( "2d" );
		window.onresize = onResize;
		window.onmousedown = onMouseDown;
		onResize();
		update()
		
	}
	
	function onResize( e )
	{
		
		reset();
		
	}
	
	function getWidth( element ){return Math.max(element.scrollWidth,element.offsetWidth,element.clientWidth );}
	function getHeight( element ){return Math.max(element.scrollHeight,element.offsetHeight,element.clientHeight );}
	
	function reset()
	{
		axes = [];
		
		width = c.width = window.innerWidth
    height = c.height = window.innerHeight
		
		var random = 500;
		
		var cols = 30;
		for( var i = 0; i < cols; i++ )
		{
			var a = new point( width / cols * i + random * Math.random(), 0 );
			var b = new point( width / cols * i + random * Math.random(), height );
			axes.push( new Axe( a,b ) );
		}
		
		var rows = 20;
		for( var i = 0; i < rows; i++ )
		{
			var a = new point( 0, 		height / rows * i + random * Math.random());
			var b = new point( width, 	height / rows * i + random * Math.random());
			axes.push( new Axe( a,b ) );
		}
    wanderers = [];
		for( var i = 0; i < colors.length * 3; i++ )
		{
			wanderers.push( new wanderer( Math.random() * width, Math.random() * height, 
                                   0,
                                   Math.random() + .5, colors[ i % colors.length ]) );
		}
		
	}
	
	function onMouseDown()
	{
		reset();
	}
	
	function update()
	{
		requestAnimationFrame( update )
		ctx.fillStyle = "rgba(255,255,255,.25)";
    ctx.globalCompositeOperation = "lighten";
		ctx.fillRect( 0,0,width, height );
    ctx.globalCompositeOperation = "source-over";
		
		ctx.strokeStyle = "rgba(16,16,16,.01 )";
		for( var j = 0; j < axes.length; j++ )axes[ j ].draw( ctx );
			
		for( var i = 0; i < wanderers.length; i++ )
		{
      
			var hull = [];
			var pp_hull = [];
			var p = wanderers[ i ];
			p.update( width, height );
			
			ctx.beginPath();
      
			  ctx.fillStyle = p.color;
			  ctx.arc( p.x, p.y, 8, 0, Math.PI * 2, true );
			  ctx.fill();
      
			ctx.closePath();
			
			for( var j = 0; j < axes.length; j++ )
			{
				
				var axe = axes[ j ];
        
				var r = axe.reflect( p );
				
				ctx.beginPath();
					
					ctx.fillStyle = p.color;
					ctx.arc( r.x, r.y, 2, 0, Math.PI * 2, true );
					ctx.fill();
					
				ctx.closePath();
				
			}
		}
	}
	
	var Axe = function( a,b ){ this.a = a; this.b=b; }
	Axe.prototype = 	
	{
		draw : function( ctx ){ctx.beginPath();ctx.moveTo( this.a.x, this.a.y );ctx.lineTo( this.b.x, this.b.y );ctx.stroke();ctx.closePath();}
		,reflect : function( p ){return utils.reflect( p, this.a, this.b );}
	}
	
	var wanderer = function( x,y,a,s,color ){this.x = x||0;this.y = y||0;this.a = a||0;this.s = s||1;this.color = color||"#000";}
	wanderer.prototype = 
	{
		update : function( width, height )
		{
			with( this )
			{
				a += ( Math.random() - .5 ) * 10 / 180 * Math.PI;
				
				x += Math.cos( a ) * s;
				y += Math.sin( a ) * s;
				
				if( x < 0
				||	y < 0
				||	x > width 
				||	y > height ) a += Math.PI / 180;
			}
		}
	}
	
	var point = function( x,y ){this.x = x||0;this.y = y||0;}
	var utils = function(){};
	utils.reflect = function(p,a,b)
	{
		var pp = utils.project( p, a, b, false );
		return new point( p.x + ( pp.x - p.x ) * 2,p.y + ( pp.y - p.y ) * 2  );
	}
	utils.project = function( p, a, b, asSegment )
	{
		var dx = b.x - a.x;
		var dy = b.y - a.y;
		if ( asSegment && dx == 0 && dy == 0 ){return a;}
		var t = ( ( p.x - a.x ) * dx + ( p.y - a.y ) * dy) / ( dx * dx + dy * dy );
		if ( asSegment && t < 0) return a;
		if ( asSegment && t > 1) return b;
		return new point( a.x + t * dx, a.y + t * dy );
	}
	
})();

})