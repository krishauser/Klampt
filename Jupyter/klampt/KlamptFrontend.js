///Uses require.js, and depends on THREE (three.js)
///
///API:
///frontend = new KLAMPT.KlamptFrontend(sceneArea);        //constructor
///KlamptFrontend.resize(w,h);           //set the width/height
///KlamptFrontend.set_shadow(enabled);   //turns shadows on/off
///KlamptFrontend.update_scene(scene);   //from a JSON message either requesting a whole scene, just a scene update, and possible RPC calls, updates the scene
///KlamptFrontend.reset_scene();         //deletes evertyhing in the scene
///KlamptFrontend.rpc(request);          //performs an RPC call from a kviz request object
///KlamptFrontend.get_camera();          ///returns the current camera
///KlamptFrontend.reset_camera();
///
///RPC calls are designed to be idempotent.
///
///Current RPC calls:
///(note: ? that updates can be called with or without the given item.  =value indicates that a default value is used if the argument is not specified.)
///- set_camera(position?,target?,up?,near?,far?);
///- clear_extras()
///- remove(object)
///- set_color(object,rgba)
///- set_visible(object,value)
///- set_transform(object,matrix)
///- add_text(name,x?,y?,text?)
///- add_ghost(prefix_name,object)
///- add_sphere(name,x?,y?,z?,r?)
///- add_line(name,verts,width=1)
///- add_xform(name,length,width=1)
///- add_trilist(name,verts)
///- add_trimesh(name,verts,indices)
///- add_points(name,verts,colors=null,size=1)
///- add_billboard(name,image,size,filter,colormap)
///    OR
///  add_billboard(name,imagedata,w,h,size,filter,colormap)

define(["nbextensions/klampt/three.min"], function(THREE) {

/**
 * modified THREE.TrackballControls
 * @author Eberhard Graether / http://egraether.com/
 * @author Mark Lundin  / http://mark-lundin.com
 * @author Simone Manini / http://daron1337.github.io
 * @author Luca Antiga  / http://lantiga.github.io
 *
 * Modified to suit KlamptFrontend
 */


THREE.TrackballControls = function ( object, domElement ) {

  var _this = this;
  var STATE = { NONE: - 1, ROTATE: 0, ZOOM: 1, PAN: 2, TOUCH_ROTATE: 3, TOUCH_ZOOM_PAN: 4 };

  this.object = object;
  this.domElement = ( domElement !== undefined ) ? domElement : document;

  // API

  this.enabled = true;

  this.screen = { left: 0, top: 0, width: 0, height: 0 };

  this.rotateSpeed = 1.0;
  this.zoomSpeed = 1.2;
  this.panSpeed = 0.3;

  this.noRotate = false;
  this.noZoom = false;
  this.noPan = false;

  this.staticMoving = false;
  this.dynamicDampingFactor = 0.2;

  this.minDistance = 0;
  this.maxDistance = Infinity;

  // internals

  this.target = new THREE.Vector3();

  //var EPS = 0.000001;
  var EPS = 0.0001;

  var lastPosition = new THREE.Vector3();

  var _state = STATE.NONE,
  _prevState = STATE.NONE,

  _eye = new THREE.Vector3(),

  _movePrev = new THREE.Vector2(),
  _moveCurr = new THREE.Vector2(),

  _lastAxis = new THREE.Vector3(),
  _lastAngle = 0,

  _zoomStart = new THREE.Vector2(),
  _zoomEnd = new THREE.Vector2(),

  _touchZoomDistanceStart = 0,
  _touchZoomDistanceEnd = 0,

  _panStart = new THREE.Vector2(),
  _panEnd = new THREE.Vector2();

  // for reset

  this.target0 = this.target.clone();
  this.position0 = this.object.position.clone();
  this.up0 = this.object.up.clone();

  // events

  var changeEvent = { type: 'change' };
  var startEvent = { type: 'start' };
  var endEvent = { type: 'end' };


  // methods

  this.handleResize = function () {

    if ( this.domElement === document ) {

      this.screen.left = 0;
      this.screen.top = 0;
      this.screen.width = window.innerWidth;
      this.screen.height = window.innerHeight;

    } else {

      var box = this.domElement.getBoundingClientRect();
      // adjustments come from similar code in the jquery offset() function
      var d = this.domElement.ownerDocument.documentElement;
      this.screen.left = box.left + window.pageXOffset - d.clientLeft;
      this.screen.top = box.top + window.pageYOffset - d.clientTop;
      this.screen.width = box.width;
      this.screen.height = box.height;

    }

  };

  this.handleEvent = function ( event ) {

    if ( typeof this[ event.type ] == 'function' ) {

      this[ event.type ]( event );

    }

  };

  var getMouseOnScreen = ( function () {

    var vector = new THREE.Vector2();

    return function getMouseOnScreen( pageX, pageY ) {

      vector.set(
        ( pageX - _this.screen.left ) / _this.screen.width,
        ( pageY - _this.screen.top ) / _this.screen.height
      );

      return vector;

    };

  }() );

  var getMouseOnCircle = ( function () {

    var vector = new THREE.Vector2();

    return function getMouseOnCircle( pageX, pageY ) {

      vector.set(
        ( ( pageX - _this.screen.width * 0.5 - _this.screen.left ) / ( _this.screen.width * 0.5 ) ),
        ( ( _this.screen.height + 2 * ( _this.screen.top - pageY ) ) / _this.screen.width ) // screen.width intentional
      );

      return vector;

    };

  }() );

  this.rotateCamera = ( function() {

    var axis = new THREE.Vector3(),
      quaternion = new THREE.Quaternion(),
      eyeDirection = new THREE.Vector3(),
      objectUpDirection = new THREE.Vector3(),
      objectSidewaysDirection = new THREE.Vector3(),
      moveDirection = new THREE.Vector3(),
      angle;

    return function rotateCamera() {

      moveDirection.set( _moveCurr.x - _movePrev.x, _moveCurr.y - _movePrev.y, 0 );
      angle = moveDirection.length();

      if ( angle ) {

        _eye.copy( _this.object.position ).sub( _this.target );

        eyeDirection.copy( _eye ).normalize();
        objectUpDirection.copy( _this.object.up ).normalize();
        objectSidewaysDirection.crossVectors( objectUpDirection, eyeDirection ).normalize();

        objectUpDirection.setLength( _moveCurr.y - _movePrev.y );
        objectSidewaysDirection.setLength( _moveCurr.x - _movePrev.x );

        moveDirection.copy( objectUpDirection.add( objectSidewaysDirection ) );

        axis.crossVectors( moveDirection, _eye ).normalize();

        angle *= _this.rotateSpeed;
        quaternion.setFromAxisAngle( axis, angle );

        _eye.applyQuaternion( quaternion );
        _this.object.up.applyQuaternion( quaternion );

        _lastAxis.copy( axis );
        _lastAngle = angle;

      } else if ( ! _this.staticMoving && _lastAngle ) {

        _lastAngle *= Math.sqrt( 1.0 - _this.dynamicDampingFactor );
        _eye.copy( _this.object.position ).sub( _this.target );
        quaternion.setFromAxisAngle( _lastAxis, _lastAngle );
        _eye.applyQuaternion( quaternion );
        _this.object.up.applyQuaternion( quaternion );

      }

      _movePrev.copy( _moveCurr );

    };

  }() );


  this.zoomCamera = function () {

    var factor;

    if ( _state === STATE.TOUCH_ZOOM_PAN ) {

      factor = _touchZoomDistanceStart / _touchZoomDistanceEnd;
      _touchZoomDistanceStart = _touchZoomDistanceEnd;
      _eye.multiplyScalar( factor );

    } else {

      factor = 1.0 + ( _zoomEnd.y - _zoomStart.y ) * _this.zoomSpeed;

      if ( factor !== 1.0 && factor > 0.0 ) {

        _eye.multiplyScalar( factor );

        if ( _this.staticMoving ) {

          _zoomStart.copy( _zoomEnd );

        } else {

          _zoomStart.y += ( _zoomEnd.y - _zoomStart.y ) * this.dynamicDampingFactor;

        }

      }

    }

  };

  this.panCamera = ( function() {

    var mouseChange = new THREE.Vector2(),
      objectUp = new THREE.Vector3(),
      pan = new THREE.Vector3();

    return function panCamera() {

      mouseChange.copy( _panEnd ).sub( _panStart );

      if ( mouseChange.lengthSq() ) {

        mouseChange.multiplyScalar( _eye.length() * _this.panSpeed );

        pan.copy( _eye ).cross( _this.object.up ).setLength( mouseChange.x );
        pan.add( objectUp.copy( _this.object.up ).setLength( mouseChange.y ) );

        _this.object.position.add( pan );
        _this.target.add( pan );

        if ( _this.staticMoving ) {

          _panStart.copy( _panEnd );

        } else {

          _panStart.add( mouseChange.subVectors( _panEnd, _panStart ).multiplyScalar( _this.dynamicDampingFactor ) );

        }

      }

    };

  }() );

  this.checkDistances = function () {

    if ( ! _this.noZoom || ! _this.noPan ) {

      if ( _eye.lengthSq() > _this.maxDistance * _this.maxDistance ) {

        _this.object.position.addVectors( _this.target, _eye.setLength( _this.maxDistance ) );
        _zoomStart.copy( _zoomEnd );

      }

      if ( _eye.lengthSq() < _this.minDistance * _this.minDistance ) {

        _this.object.position.addVectors( _this.target, _eye.setLength( _this.minDistance ) );
        _zoomStart.copy( _zoomEnd );

      }

    }

  };

  this.update = function () {

    _eye.subVectors( _this.object.position, _this.target );

    if ( ! _this.noRotate ) {

      _this.rotateCamera();

    }

    if ( ! _this.noZoom ) {

      _this.zoomCamera();

    }

    if ( ! _this.noPan ) {

      _this.panCamera();

    }

    _this.object.position.addVectors( _this.target, _eye );

    _this.checkDistances();

    _this.object.lookAt( _this.target );

    if ( lastPosition.distanceToSquared( _this.object.position ) > EPS ) {

      _this.dispatchEvent( changeEvent );

      lastPosition.copy( _this.object.position );

    }

  };

  this.reset = function () {

    _state = STATE.NONE;
    _prevState = STATE.NONE;

    _this.target.copy( _this.target0 );
    _this.object.position.copy( _this.position0 );
    _this.object.up.copy( _this.up0 );

    _eye.subVectors( _this.object.position, _this.target );

    _this.object.lookAt( _this.target );

    _this.dispatchEvent( changeEvent );

    lastPosition.copy( _this.object.position );

  };

  
  function mousedown( event ) {
    if ( _this.enabled === false ) return;

    event.preventDefault();
    event.stopPropagation();

    if ( _state === STATE.NONE ) {

      _state = event.button;
      //use shift / control, especially on mac
      if(_state == STATE.ROTATE) {
        if(event.shiftKey) {
          _state = STATE.ZOOM;
        }
        else if(event.ctrlKey) {
          _state = STATE.PAN;
        }
      }
    }

    if ( _state === STATE.ROTATE && ! _this.noRotate ) {

      _moveCurr.copy( getMouseOnCircle( event.pageX, event.pageY ) );
      _movePrev.copy( _moveCurr );

    } else if ( _state === STATE.ZOOM && ! _this.noZoom ) {

      _zoomStart.copy( getMouseOnScreen( event.pageX, event.pageY ) );
      _zoomEnd.copy( _zoomStart );

    } else if ( _state === STATE.PAN && ! _this.noPan ) {

      _panStart.copy( getMouseOnScreen( event.pageX, event.pageY ) );
      _panEnd.copy( _panStart );

    }

    _this.domElement.addEventListener( 'mousemove', mousemove, false );
    _this.domElement.addEventListener( 'mouseup', mouseup, false );

    _this.dispatchEvent( startEvent );

  }

  function mousemove( event ) {

    if ( _this.enabled === false ) return;

    event.preventDefault();
    event.stopPropagation();

    if ( _state === STATE.ROTATE && ! _this.noRotate ) {

      _movePrev.copy( _moveCurr );
      _moveCurr.copy( getMouseOnCircle( event.pageX, event.pageY ) );

    } else if ( _state === STATE.ZOOM && ! _this.noZoom ) {

      _zoomEnd.copy( getMouseOnScreen( event.pageX, event.pageY ) );

    } else if ( _state === STATE.PAN && ! _this.noPan ) {

      _panEnd.copy( getMouseOnScreen( event.pageX, event.pageY ) );

    }
    _this.update();
  }

  function mouseup( event ) {
    if ( _this.enabled === false ) return;

    event.preventDefault();
    event.stopPropagation();

    _state = STATE.NONE;

    _this.domElement.removeEventListener( 'mousemove', mousemove );
    _this.domElement.removeEventListener( 'mouseup', mouseup );
    _this.dispatchEvent( endEvent );

  }

  function mousewheel( event ) {
    if ( _this.enabled === false ) return;

    event.preventDefault();
    event.stopPropagation();

    var delta = 0;

    if ( event.wheelDelta ) {

      // WebKit / Opera / Explorer 9

      delta = event.wheelDelta / 40;

    } else if ( event.detail ) {

      // Firefox

      delta = - event.detail / 3;

    }

    _zoomStart.y += delta * 0.01;
    _this.dispatchEvent( startEvent );
    _this.dispatchEvent( endEvent );
    _this.update();
  }

  function touchstart( event ) {

    if ( _this.enabled === false ) return;

    switch ( event.touches.length ) {

      case 1:
        _state = STATE.TOUCH_ROTATE;
        _moveCurr.copy( getMouseOnCircle( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY ) );
        _movePrev.copy( _moveCurr );
        break;

      default: // 2 or more
        _state = STATE.TOUCH_ZOOM_PAN;
        var dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
        var dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
        _touchZoomDistanceEnd = _touchZoomDistanceStart = Math.sqrt( dx * dx + dy * dy );

        var x = ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX ) / 2;
        var y = ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY ) / 2;
        _panStart.copy( getMouseOnScreen( x, y ) );
        _panEnd.copy( _panStart );
        break;

    }

    _this.dispatchEvent( startEvent );

  }

  function touchmove( event ) {

    if ( _this.enabled === false ) return;

    event.preventDefault();
    event.stopPropagation();

    switch ( event.touches.length ) {

      case 1:
        _movePrev.copy( _moveCurr );
        _moveCurr.copy( getMouseOnCircle( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY ) );
        break;

      default: // 2 or more
        var dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
        var dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
        _touchZoomDistanceEnd = Math.sqrt( dx * dx + dy * dy );

        var x = ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX ) / 2;
        var y = ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY ) / 2;
        _panEnd.copy( getMouseOnScreen( x, y ) );
        break;

    }
    _this.update();
  }

  function touchend( event ) {

    if ( _this.enabled === false ) return;

    switch ( event.touches.length ) {

      case 0:
        _state = STATE.NONE;
        break;

      case 1:
        _state = STATE.TOUCH_ROTATE;
        _moveCurr.copy( getMouseOnCircle( event.touches[ 0 ].pageX, event.touches[ 0 ].pageY ) );
        _movePrev.copy( _moveCurr );
        break;

    }

    _this.dispatchEvent( endEvent );

  }

  function contextmenu( event ) {

    event.preventDefault();

  }

  this.dispose = function() {

    this.domElement.removeEventListener( 'contextmenu', contextmenu, false );
    this.domElement.removeEventListener( 'mousedown', mousedown, false );
    this.domElement.removeEventListener( 'mousewheel', mousewheel, false );
    this.domElement.removeEventListener( 'MozMousePixelScroll', mousewheel, false ); // firefox

    this.domElement.removeEventListener( 'touchstart', touchstart, false );
    this.domElement.removeEventListener( 'touchend', touchend, false );
    this.domElement.removeEventListener( 'touchmove', touchmove, false );

    this.domElement.removeEventListener( 'mousemove', mousemove, false );
    this.domElement.removeEventListener( 'mouseup', mouseup, false );
  };

  this.domElement.addEventListener( 'contextmenu', contextmenu, false );
  this.domElement.addEventListener( 'mousedown', mousedown, false );
  this.domElement.addEventListener( 'mousewheel', mousewheel, false );
  this.domElement.addEventListener( 'MozMousePixelScroll', mousewheel, false ); // firefox

  this.domElement.addEventListener( 'touchstart', touchstart, false );
  this.domElement.addEventListener( 'touchend', touchend, false );
  this.domElement.addEventListener( 'touchmove', touchmove, false );

  this.handleResize();

  // force an update at start
  this.update();

};

THREE.TrackballControls.prototype = Object.create( THREE.EventDispatcher.prototype );
THREE.TrackballControls.prototype.constructor = THREE.TrackballControls;




function KlamptFrontend(dom_sceneArea) {

  var _this = this;

  function is_undefined_or_null(x) {
     return (typeof x === 'undefined' || x === null);
  }

  this.set_shadow = function(enabled)
  {
    this.renderer.shadowMap.enabled = enabled;
  }

  this.resize = function( w,h )
  {
    console.log("KLAMPT.resize width: " + w + " height: " + h);

    this.width= w; //account for 5px padding on each side
    this.height=h;

    this.renderer.setSize(this.width,this.height);
    this.camera.aspect = this.width/ this.height;

    this.camera.updateProjectionMatrix();  
    this.controls.handleResize();
    this.render();
  }

  function addObject(name,object)
  {
    object.name = name;
    _this.sceneCache[name] = object;
    _this.extras.add(object);
  }

  function getObject(name) 
  {
    var object = _this.sceneCache[name];
    if(object == null) {
      object = _this.scene.getObjectByName(name, true );
      if(object != null) {
        _this.sceneCache[name] = object;
      }
      return object;
    }
    return object;
  }

  function removeObject(name)
  {
    var obj = getObject(name);
    if(obj) {
       if(name in _this.sceneCache) {
          delete _this.sceneCache[name];
       }
       if ( !is_undefined_or_null(obj.geometry) ) obj.geometry.dispose();
       if ( !is_undefined_or_null(obj.material) ) obj.material.dispose();
       obj.visible = false;
       obj.parent.remove(obj);
    }
    else {
      obj = _this.sceneArea.querySelector("#_text_overlay_"+name);
      if(obj) {
        obj.parentNode.removeChild(obj);
      }
      else {
        console.log("KLAMPT.rpc: Item to be removed "+name+" not found");
      }
    }
  }

  //dataJ has a Three.js scene object format
  this._set_scene = function (dataJ)
  {
     this.scene.traverse( function ( child ) { //make sure to dispose all old objects
          if ( !is_undefined_or_null(child.geometry) ) child.geometry.dispose();
         if ( !is_undefined_or_null(child.material) ) child.material.dispose();
     } );
     this.scene=null;
     this.sceneCache={};

     this.scene = this.loader.parse( dataJ );
     if (this.scene == null) {
       console.log("KLAMPT.update_scene: Invalid scene sent from server");
       this.scene = new THREE.Scene();
     }

     var newItems = [];
     this.scene.traverse( function (child) {
      if(!(child instanceof THREE.Light)) {
        if ( !is_undefined_or_null(child.geometry) ) {
          if(child.geometry instanceof THREE.Geometry) {
            child.geometry.computeFaceNormals();
            console.log("Geometry: normals of "+child.name+" calculated\n");
          }
          else if(child instanceof THREE.Points) {
            //no normals needed here
          }
          else if(child.geometry instanceof THREE.BufferGeometry) {
            if(is_undefined_or_null(child.geometry.attributes.normal)) {  //need to compute normals
              console.log("BufferGeometry: Computing normals of "+child.name+" from triangles\n");
              var positions = child.geometry.attributes["position"];
              var indices = child.geometry.index.array;
              var normals = new Float32Array(positions.array.length);
              for(var i=0;i<positions.array.length;i++)
                normals[i] = 0.0;
              var vba = new THREE.Vector3();
              var vca = new THREE.Vector3();
              var vn = new THREE.Vector3();
              for(var tri=0;tri<indices.length;tri+=3) {
                var a=indices[tri];
                var b=indices[tri+1];
                var c=indices[tri+2];
                vba.x = positions.array[b*3]-positions.array[a*3];
                vba.y = positions.array[b*3+1]-positions.array[a*3+1];
                vba.z = positions.array[b*3+2]-positions.array[a*3+2];
                vca.x = positions.array[c*3]-positions.array[a*3];
                vca.y = positions.array[c*3+1]-positions.array[a*3+1];
                vca.z = positions.array[c*3+2]-positions.array[a*3+2];
                vn.crossVectors(vba,vca);
                vn.normalize();
                normals[a*3] += vn.x;
                normals[a*3+1] += vn.y;
                normals[a*3+2] += vn.z;
                normals[b*3] += vn.x;
                normals[b*3+1] += vn.y;
                normals[b*3+2] += vn.z;
                normals[c*3] += vn.x;
                normals[c*3+1] += vn.y;
                normals[c*3+2] += vn.z;
              }
              for(var i=0;i<positions.array.length;i+=3) {
                vn.x = normals[i];
                vn.y = normals[i+1];
                vn.z = normals[i+2];
                vn.normalize();
                normals[i] = vn.x;
                normals[i+1] = vn.y;
                normals[i+2] = vn.z;
              }
              child.geometry.setAttribute( 'normal', new THREE.BufferAttribute( normals, 3 ) );
            }
            else {
              console.log("BufferGeometry Item "+child.name+" already has normals\n");
            }
          }
          else {
            console.log("Item "+child.name+" is neither a geometry or THREE.BufferGeometry\n");
          }
        }
        if(child.name == "Terrain") {
          child.receiveShadow = true;
          child.castShadow = true;
        }
        else {
          child.receiveShadow = true;
          child.castShadow = true;
        }

      }
      else if(child instanceof THREE.DirectionalLight || child instanceof THREE.SpotLight) {
        child.intensity *= 0.8;
        child.castShadow = true;
        //child.shadow.darkness = 0.3;
        if(child instanceof THREE.DirectionalLight) {
          //child.position.set( 0, 0, 10 ); 
          //child.shadow.camera.fov = 50;
          child.shadow.bias = -0.00001;
          child.shadow.mapSize.x = 1024;
          child.shadow.mapSize.y = 1024;
          child.shadow.camera.right     =  5;
          child.shadow.camera.left     = -5;
          child.shadow.camera.top      =  5;
          child.shadow.camera.bottom   = -5;
          /*
          var helper = new THREE.CameraHelper( light.shadow.camera );
          newItems.push( helper );

          //hack for non-black shadows
          var shadowIntensity = 0.3;
          var light2 = child.clone();
          child.castShadow = false;
          light2.intensity = shadowIntensity;
          child.intensity = child.intensity - shadowIntensity;
          newItems.push(light2);
          */
        }
      }
     });
     this.scene.add(new THREE.AmbientLight(0xffffff,0.2));
     for(var i=0;i<newItems.length;i++)
        this.scene.add(newItems[i]);


    var AxesHelper = new THREE.AxesHelper( 0.2 );
    AxesHelper.material.linewidth = 2.0;
    this.scene.add( AxesHelper );
    this.extras = new THREE.Group();
    this.scene.add(this.extras);
  }

  ///sceneObjects is a list of dictionaries, each containing the members "name" and "matrix"
  this._set_transforms = function (sceneObjects)
  {
     for(i=0; i<sceneObjects.length; i++)
     {  
        //console.log("Update requested to: " + sceneObjects[i].name);
        //console.log("  new matrix is: " + sceneObjects[i].matrix);

        var object = getObject(sceneObjects[i].name);
        if(object != null)
        { 
          //console.log("  we found \"" + sceneObjects[i].name + "\" in the Three.js scene");
                         
          object.matrixAutoUpdate=false;
          object.matrixWorldNeedsUpdate=true;
        
          var m=sceneObjects[i].matrix;
     
          object.matrix.set(m[0],m[4],m[8],m[12],m[1],m[5],m[9],m[13],m[2],m[6],m[10],m[14],m[3],m[7],m[11],m[15]);
        } 
        else {
          console.log("KLAMPT.update_scene: Did not find \"" + sceneObjects[i].name + "\" in the Three.js scene");
        }
     } 
  }


  function _power_of_2(number) {
    return (number != 0) && (number & (number - 1)) == 0;
  }

  function _setupBillboard(name,texture,size) {
    var material;
    if (!_power_of_2(texture.image.width) || !_power_of_2(texture.image.height)) {
      console.log("Warning, texture does not have a power of two width / height: "+texture.image.width+","+texture.image.height);
      return;
    }
    else {
      if(texture.format == THREE.AlphaFormat || texture.format == THREE.LuminanceFormat ) {
        material = new THREE.MeshBasicMaterial( {
          alphaMap : texture,
          transparent: true,
          opacity: 1
        } );
      }
      else {
        material = new THREE.MeshBasicMaterial( {
          map: texture
        } );
      }
    }

    var obj = getObject(name);
    obj.material = material;
    obj.material.needsUpdate = true;
  }

  function clone_material(mat)
  {
    var res;
    if(mat instanceof THREE.MeshPhongMaterial)
      res = new THREE.MeshPhongMaterial();
    else if(mat instanceof THREE.LineBasicMaterial)
      res = new THREE.LineBasicMaterial();
    else
      res = new THREE.MeshBasicMaterial();
    res.clone(mat);
    return res;
  }

  this.rpc = function(request)
  {
     if(request.type == 'set_camera') {
      var data=request;
      if(data.up !== undefined) {
        this.camera.up.x = data.up.x;
        this.camera.up.y = data.up.y;
        this.camera.up.z = data.up.z;
      }
      if(data.target !== undefined) {
        this.controls.target.x = data.target.x;
        this.controls.target.y = data.target.y;
        this.controls.target.z = data.target.z;
      }
      if(data.position !== undefined) {
        this.camera.position.x = data.position.x;
        this.camera.position.y = data.position.y;
        this.camera.position.z = data.position.z;
      }
      if(data.near !== undefined) {
        this.camera.near = data.near;
      }
      if(data.far !== undefined) {
        this.camera.far = data.far;
      }
      this.controls.update();
     }
     else if(request.type == "clear_extras")  {
       //clear scene
       toclear = [];
       this.extras.traverse(function(child) {
          if(!is_undefined_or_null(child.name) && child.name in _this.sceneCache) 
            delete _this.sceneCache[child.name];
          if ( !is_undefined_or_null(child.geometry) ) child.geometry.dispose();
          if ( !is_undefined_or_null(child.material) ) child.material.dispose();
       } );
       this.scene.remove(this.extras);
       this.extras = new THREE.Group();
       this.scene.add(this.extras);
       //clear text
       var overlayList = [];
       for(var i=0;i<this.sceneArea.children.length; i++) {
        if(this.sceneArea.children[i].id.startsWith("_text_overlay_")) {
          overlayList.push(this.sceneArea.children[i]);
          //console.log("Removing text item "+sceneArea.children[i].id);
        }
      }
      for (i=0;i<overlayList.length;i++) {
        //console.log("Clearing text "+overlayList[i].id);
        this.sceneArea.removeChild(overlayList[i]);
      }
     }
     else if(request.type == "remove") {
       //remove object from scene
       //console.log("Removing item "+request.object);
       removeObject(request.object);
     }
     else if(request.type == "set_color") 
     {
        var object_name=request.object;
        var rgba=request.rgba;
        var recursive=request.recursive;
                                                   
        //console.log("set_color requested. object: " + object_name + " rgba: " + rgba); 
        
        var obj = getObject(object_name);
        if(obj == null) {
          console.log("KLAMPT.rpc: Invalid object name "+object_name+" specified in set_color");
        }
        else { 
          var shared = (is_undefined_or_null(obj.userData.customSharedMaterialSetup));
           //if(!is_undefined_or_null(typeof object.material))
           //{
            //  console.log("first checking if we've working this this material before");
                                                              
              if (recursive == true)
              {
                if(is_undefined_or_null(obj.material)) {
                  obj.material=new THREE.MeshPhongMaterial();
                  obj.userData.customSharedMaterialSetup=true;
                }
                else if(shared)
                {
                  obj.material = clone_material(obj.material);                  
                  obj.userData.customSharedMaterialSetup=true;
                }
                obj.traverse( function ( child ) { 
                  if (!is_undefined_or_null(child.material)) {
                    var cshared = (is_undefined_or_null(child.userData.customSharedMaterialSetup));
                    if(!cshared) { child.material.dispose(); }
                    child.material=obj.material;
                  }
                } );
              }
              else
              {
                if(obj.material == null) {
                  obj.material=new THREE.MeshPhongMaterial();
                  obj.userData.customSingleMaterialSetup=true;
                }
                else if(shared)
                { 
                  obj.material=clone_material(obj.material);
                  obj.userData.customSingleMaterialSetup=true;
                }
              }
              

              obj.material.color.setRGB(rgba[0],rgba[1],rgba[2]);
              if(rgba[3]!=1.0)
              {
                 obj.material.transparent=true;
                 obj.material.opacity=rgba[3];
              }
              else
              {
                if(obj.material.alphaMap != null) 
                  obj.material.transparent=true;
                else
                  obj.material.transparent=false;
              }
           //}
           //else
           //{
           //   console.log("ERROR: no material associated with object: " + object_name);  
           //   alert("ERROR: kviz.set_color is trying to set an object with no material");
           //}
        }
     }
     else if(request.type == "set_visible") 
     {
        var object_name=request.object;
        var visible=request.value;
                                                   
        //console.log("set_visible requested. object: " + object_name + " visible: " + visible); 
        
        var object = getObject(object_name);
        if(object == null) {
          console.log("KLAMPT.rpc: Invalid object name "+object_name+" specified in set_visible");
        }
        else {
          object.visible = visible;
        }
     }
     else if(request.type == "add_ghost") 
     {
        var object_name=request.object;
        var prefix=request.prefix_name;
                                                   
        //console.log("add_ghost requested. object: " + object_name + " prefix: " + prefix); 
        var old_ghost = getObject(prefix+object_name);
        if(old_ghost == null) { 
          var object = getObject(object_name);
          if(object != null)
          { 
             var clone_object=object.clone(true);
             clone_object.traverse( function ( child ) { 
                      if (!is_undefined_or_null(child.name)) {
                         child.name=prefix+child.name;
                      }
                      //ghosts should not cast shadows
                      if (!is_undefined_or_null(child.castShadow)) {
                         child.castShadow = false;
                         child.receiveShadow = false;
                      }
             });
             addObject(prefix+object_name,clone_object);
             console.log("KLAMPT.rpc: Added ghost with name "+prefix+object_name);
          }
          else {
             console.log("KLAMPT.rpc: The ghost of object " + object_name + " could not be made since the object was not found");
          }
        }
        else {
          //there's already a ghost with that name... should we re-clone?
        }
     }
     else if(request.type == "set_transform")
     {                 
        //console.log("KLAMPT.rpc: got a set_transform RPC request for: " + request.object);
        var object = getObject(request.object);
        if(object != null)
        {
          if(object.matrix) {
            object.matrixAutoUpdate=false;
            object.matrixWorldNeedsUpdate=true;
          
            var m=request.matrix;
            object.matrix.set(m[0],m[1],m[2],m[3],m[4],m[5],m[6],m[7],m[8],m[9],m[10],m[11],m[12],m[13],m[14],m[15]);
          } 
          else 
            console.log("KLAMPT.rpc: object does not have matrix property: " + request.object);
        }
        else
           console.log("KLAMPT.rpc: couldn't find object: " + request.object);
     }
     else if(request.type == "add_text")
     {
        //console.log("RPC to add text!");   
        var text2 = this.sceneArea.querySelector("#_text_overlay_"+request.name);
        if(text2 == null) {
          var text2 = document.createElement('div');
          text2.style.position = 'absolute';
          text2.id="_text_overlay_"+request.name;
          text2.style.zIndex = 1;    // if you still don't see the label, try uncommenting this
          //text2.style.width = 100;
          //text2.style.height = 100;
          //text2.style.backgroundColor = "blue";
          if(request.text!=null)
             text2.innerHTML = request.text;
             
          text2.style.top = request.y + '%';
          text2.style.left = request.x + '%';
          this.sceneArea.appendChild(text2);
        }
        else {
          if(!is_undefined_or_null(request.text))
            text2.innerHTML = request.text;
          if(!is_undefined_or_null(request.x))
            text2.style.left = request.x + '%';
          if(!is_undefined_or_null(request.y))
            text2.style.top = request.y + '%';
        }
     }
     else if(request.type == "add_sphere")
     {
        var sphere = getObject(request.name);
        if(sphere == null) {
          //console.log("RPC to add sphere!"); 
          slices = 20;
          if(request.r < 0.05) slices = 6;
          else if(request.r < 0.2) slices = 12;

          var geometry = new THREE.SphereGeometry(1.0,slices,slices);
          var material = new THREE.MeshPhongMaterial( {color: 0xAA0000} );
          var sphere = new THREE.Mesh( geometry, material );
          sphere.userData.customSharedMaterialSetup=true;
          sphere.castShadow = true;
          
          sphere.scale.x=request.r;
          sphere.scale.y=request.r;
          sphere.scale.z=request.r;
          
          sphere.position.set(request.x,request.y,request.z);
          addObject(request.name,sphere);
        }
        else { 
           if(!is_undefined_or_null(request.x)) {
             sphere.position.set(request.x,request.y,request.z);
           }
           if(!is_undefined_or_null(request.r) && request.r > 0)
           {
              sphere.scale.x=request.r;
              sphere.scale.y=request.r;
              sphere.scale.z=request.r;
           }
        }
     }
     else if(request.type == "add_line")
     {
        var line = getObject(request.name);
        if(line == null) {
          var geometry = new THREE.Geometry();
          
          for(var i=0;i<request.verts.length;i+=3) {
            geometry.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
          }
          geometry.dynamic  = true;
             
          var material = new THREE.LineBasicMaterial( {color: 0xAA0000} );
          if(!is_undefined_or_null(request.width)) {
            material.linewidth = request.width;
          }
          var line = new THREE.Line( geometry, material );
          line.userData.customSharedMaterialSetup=true;
          addObject(request.name,line);
        }
        else {
           line.geometry.vertices = []
           for(var i=0;i<request.verts.length;i+=3) {
             line.geometry.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
           }
           line.geometry.verticesNeedUpdate = true;
        }
     }
     else if(request.type == "add_xform")
     {
        var xform = getObject(request.name);
        if(xform != null)
          removeObject(request.name);
        var axis = new THREE.AxesHelper(request.length);
        if(!is_undefined_or_null(request.width)) axis.material.linewidth = request.width;
        addObject(request.name,axis);
     } 
     else if(request.type == 'add_trilist')
     {
       var obj = getObject(request.name);
       if(obj == null) {
         var geom = new THREE.Geometry();
         geom.dynamic = true;
         for(var i=0;i<request.verts.length;i+=3) {
            geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
         }
         for(var i=0;i<request.verts.length;i+=9) {
            geom.faces.push( new THREE.Face3( i/3, i/3+1, i/3+2 ) );
         }
         geom.computeFaceNormals();
         geom.castShadow = true;
         var mesh= new THREE.Mesh( geom, new THREE.MeshPhongMaterial() );
         mesh.userData.customSharedMaterialSetup=true;
         addObject(request.name,mesh);
        }
        else {
          if(request.verts.length != obj.geometry.vertices.length*3 || true) {
            //might as well just completely recreate the geometry
            obj.geometry.dispose();
            var geom = new THREE.Geometry();
            geom.dynamic = true;
             for(var i=0;i<request.verts.length;i+=3) {
                geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
             }
             for(var i=0;i<request.verts.length;i+=9) {
                geom.faces.push( new THREE.Face3( i/3, i/3+1, i/3+2 ) );
             }
            geom.computeFaceNormals();
            obj.geometry = geom;
          }
          else {
            //for some reason this isn't working
            //console.log("Updating trilist vertices");
            obj.geometry.dynamic = true;
            obj.geometry.verticesNeedUpdate = true;
           for(var i=0;i<request.verts.length;i+=3) {
              obj.geometry.vertices[i/3] = new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]);
           }
            obj.geometry.computeFaceNormals();
         }
       }
     }
     else if(request.type == 'add_trimesh')
     {
       var obj = getObject(request.name);
       if(obj == null) {
         var geom = new THREE.Geometry();
         geom.dynamic = true;
         for(var i=0;i<request.verts.length;i+=3) {
            geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
         }
         for(var i=0;i<request.tris.length;i+=3) {
            geom.faces.push( new THREE.Face3( request.tris[i], request.tris[i+1], request.tris[i+2] ) );
         }
         geom.computeFaceNormals();
         geom.castShadow = true;
         var mesh= new THREE.Mesh( geom, new THREE.MeshPhongMaterial() );
         mesh.userData.customSharedMaterialSetup=true;
         addObject(request.name,mesh);
        }
        else {
          if(request.verts.length != obj.geometry.vertices.length*3 || true) {
            //might as well just completely recreate the geometry
            obj.geometry.dispose();
            var geom = new THREE.Geometry();
            geom.dynamic = true;
            for(var i=0;i<request.verts.length;i+=3) {
              geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
            }
            for(var i=0;i<request.tris.length;i+=3) {
              geom.faces.push( new THREE.Face3( request.tris[i], request.tris[i+1], request.tris[i+2] ) );
            }
            geom.computeFaceNormals();
            obj.geometry = geom;
          }
          else {
            //for some reason this isn't working
            //console.log("Updating trimesh vertices");
            obj.geometry.dynamic = true;
            obj.geometry.verticesNeedUpdate = true;
            for(var i=0;i<request.verts.length;i+=3) {
               obj.geometry.vertices[i/3] = new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]);
            }
            obj.geometry.computeFaceNormals();
         }
       }
     }
     else if(request.type == 'add_points')
     {
       var obj = getObject(request.name);
       if(obj == null) {
         var geom = new THREE.Geometry();
         geom.dynamic = true;
         for(var i=0;i<request.verts.length;i+=3) {
            geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
         }
         var mat=new THREE.PointsMaterial();
         if(request.size)
           mat.size = request.size;
         else
           mat.size = 1;
         if(request.colors) {
            mat.vertexColors = true;
            for(var i=0;i<request.colors.length;i++) {
              geom.colors.push(new THREE.Color(request.colors[i]));
            }
         }
         else {
            mat.color = new THREE.Color(0xffffff);
         }
         geom.castShadow = false;
         var mesh= new THREE.Points( geom, mat );
         mesh.userData.customSharedMaterialSetup=true;
         addObject(request.name,mesh);
        }
        else {
          if(request.verts.length != obj.geometry.vertices.length*3 || true) {
            //might as well just completely recreate the geometry
            obj.geometry.dispose();
            var geom = new THREE.Geometry();
            geom.dynamic = true;
            for(var i=0;i<request.verts.length;i+=3) {
              geom.vertices.push(new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]));
            }
            if(request.colors) {
              obj.material.vertexColors = true;
              for(var i=0;i<request.colors.length;i++) {
                geom.colors.push(new THREE.Color(request.colors[i]));
              }
            }
            else {
              obj.material.vertexColors = false;
            }
            obj.geometry = geom;
          }
          else {
            //for some reason this isn't working
            //console.log("Updating point cloud vertices");
            obj.geometry.dynamic = true;
            obj.geometry.verticesNeedUpdate = true;
            for(var i=0;i<request.verts.length;i+=3) {
               obj.geometry.vertices[i/3] = new THREE.Vector3(request.verts[i],request.verts[i+1],request.verts[i+2]);
            }
            if(request.colors) {
              obj.material.vertexColors = true;
              for(var i=0;i<request.colors.length;i++) {
                obj.geometry.colors[i] = new THREE.Color(request.colors[i]);
              }
              obj.geometry.colorsNeedUpdate = true;
            }
         }
       }
     }
     else if(request.type == 'add_billboard') {
       var size = request.size;
      var geom = new THREE.Geometry();
      geom.vertices.push(new THREE.Vector3(-size[0]*0.5,-size[1]*0.5,0));
      geom.vertices.push(new THREE.Vector3(size[0]*0.5,-size[1]*0.5,0));
      geom.vertices.push(new THREE.Vector3(size[0]*0.5,size[1]*0.5,0));
      geom.vertices.push(new THREE.Vector3(-size[0]*0.5,size[1]*0.5,0));
      geom.faces.push(new THREE.Face3(0,1,2));
      geom.faces.push(new THREE.Face3(0,2,3));
      geom.faceVertexUvs[0] = [];
      geom.faceVertexUvs[0][0] = [new THREE.Vector2(0,1),new THREE.Vector2(1,1),new THREE.Vector2(1,0)];
      geom.faceVertexUvs[0][1] = [new THREE.Vector2(0,1),new THREE.Vector2(1,0),new THREE.Vector2(0,0)];
      geom.computeFaceNormals();
      geom.uvsNeedUpdate = true;
      var mesh = new THREE.Mesh( geom, new THREE.MeshBasicMaterial() );
      mesh.userData.customSharedMaterialSetup=true;
      addObject(request.name,mesh);

       var filter = request.filter;
       var colormap = request.colormap;
       if(filter == 'nearest') filter = THREE.NearestFilter;
       else filter = THREE.LinearFilter;
       if (request.imagedata) {
         //load from data
         var w=request.width;
         var h=request.height;
         var data = atob(request.imagedata);
         var format = THREE.LuminanceFormat;
         if(colormap == 'opacity')
           format = THREE.LuminanceFormat; //AlphaFormat;
         if(data.length == 3*w*h)
           format = THREE.RGBFormat;
         else if(data.length == 4*w*h)
           format = THREE.RGBAFormat;
         else  {
           if(data.length != w*h)  {
            console.log("KLAMPT.rpc: Invalid image data length? "+data.length);
            return;
          }
         }
         var buffer = new Uint8Array(new ArrayBuffer(data.length));
         for(i = 0; i < data.length; i++) {
            buffer[i] = data.charCodeAt(i);
         }
         var tex = new THREE.DataTexture(buffer,w,h,format,THREE.UnsignedByteType);
         tex.needsUpdate = true;
         //tex.minFilter = filter;
         //tex.magFilter = filter;
         _setupBillboard(request.name,tex,size);
       }
       else {
         //load from image
         // instantiate a loader
          var loader = new THREE.TextureLoader();

          // load a resource
          loader.load(
            // resource URL
            request.image,
            // Function when resource is loaded
            function ( texture ) {
              texture.minFilter = filter;
              texture.magFilter = filter;
              _setupBillboard(request.name,texture,size);
            },
            // Function called when download progresses
            function ( xhr ) {
              console.log( (xhr.loaded / xhr.total * 100) + '% loaded' );
            },
            // Function called when download errors
            function ( xhr ) {
              console.log( 'An error happened' );
            }
          );
       }
       //create the billboard geometry
     }
     else {
        console.log("KLAMPT.rpc: Invalid request: "+request.type);
     }
  }

  this.reset_scene = function()
  {
     this.scene.traverse( function ( child ) { //make sure to dispose all old objects
          if (!is_undefined_or_null(child.geometry) ) child.geometry.dispose();
         if (!is_undefined_or_null(child.material) ) child.material.dispose();
     } );
     for (let i = this.scene.children.length - 1; i >= 0; i--) {
        this.scene.remove(this.scene.children[i]);
     }
     this.sceneCache={};

     //clear anything named _text_overlay_X
     var overlayList = [];
     for(var i=0;i<this.sceneArea.children.length; i++) {
      if(this.sceneArea.children[i].id.startsWith("_text_overlay_")) {
        overlayList.push(this.sceneArea.children[i]);
        //console.log("Removing text item "+sceneArea.children[i].id);
      }
    }
    for (i=0;i<overlayList.length;i++) {
        this.sceneArea.removeChild(overlayList[i]);
    }
     var t1 = performance.now();

    this.scene.add(new THREE.AmbientLight(0xffffff,0.2));
    this.scene.add(new THREE.DirectionalLight(0xffffff,0.9));

    var AxesHelper = new THREE.AxesHelper( 0.2 );
    this.scene.add( AxesHelper );
    this.extras = new THREE.Group();
    this.scene.add(this.extras);
  }

  this.update_scene = function(data)
  {   
    //console.log("new scene has arrived!");

    //var dataJ=JSON.parse(data); 
    dataJ = data;
    if(dataJ == null) {
      console.log("KLAMPT.update_scene: Unable to parse scene JSON!");
      //console.log(data);
      return;
    }

    //need to determine if full scene or just transforms
    var isFullScene=dataJ.metadata.fullscene;

    //console.log("full scene is: " + isFullScene);

    if(isFullScene)
    {
       var t0 = performance.now();

       this._set_scene(dataJ);

       //console.log("Call to load scene " + (t1 - t0) + " milliseconds.")
       //scene.traverse ( function (child) {
       //  console.log("found: " + child.name);
       //});
    }
    else //just apply transforms
    {
       var t0 = performance.now();
       this._set_transforms(dataJ.object);
       var t1 = performance.now();
       //console.log("Call to load tranforms " + (t1 - t0) + " milliseconds.");
    }

    
    var rpc =dataJ.RPC;
    if(rpc) {
      var t1 = performance.now();
      for(i=0; i<rpc.length; i++)
      {  
        try {
          this.rpc(rpc[i]);
        }
        catch(err) {
          console.log(rpc[i]);
          throw err;
        }
      }
      var t2 = performance.now();
      if(rpc.length > 0)
      {
         //console.log("Call to do RPC's " + (t2 - t1) + " milliseconds.")
      }
    }
  }

  this.render = function()
  { 
    _this.renderer.render( _this.scene, _this.camera );
  }

  this.reset_camera = function()
  {
    if(this.camera != null)
      this.camera.dispose();
    this.camera = new THREE.PerspectiveCamera( 45, this.width/this.height, 0.1, 1000 );
    this.camera.position.z = 6;
    this.camera.position.y = 3;
    if(this.controls != null)
      this.controls.dispose();
    this.controls=new THREE.TrackballControls( this.camera, this.sceneArea);
    this.controls.rotateSpeed = 1.0;
    this.controls.zoomSpeed = 1.2;
    this.controls.panSpeed = 0.8;
    this.controls.noZoom = false;
    this.controls.noPan = false;
    this.controls.staticMoving = true;
    this.controls.dynamicDampingFactor = 0.3;
    this.controls.keys = [ 65, 83, 68 ];
    this.controls.addEventListener( 'change', this._on_camera_change );
  }

  this._on_camera_change = function() {
    _this.render();
    if(_this.cameraCallback != null) {
      //console.log("camera change, with callback");
      _this.cameraCallback();
    }
    else {
      //console.log("camera change, W/O callback");
    }
  }

  this.set_camera_callback = function(cb)
  {
    this.cameraCallback = cb;
  }

  this.get_camera = function()
  {
    return {up:this.camera.up,target:this.controls.target,position:this.camera.position,near:this.camera.near,far:this.camera.far};
  }

  this.set_camera = function(data)
  {
    this.camera.up.x = data.up.x;
    this.camera.up.y = data.up.y;
    this.camera.up.z = data.up.z;
    this.controls.target.x = data.target.x;
    this.controls.target.y = data.target.y;
    this.controls.target.z = data.target.z;
    this.camera.position.x = data.position.x;
    this.camera.position.y = data.position.y;
    this.camera.position.z = data.position.z;
    this.camera.near = data.near;
    this.camera.far = data.far;
    this.controls.update();
  }

  //the DOM element containing the scene
  this.sceneArea = dom_sceneArea;
  this.extras = null;
  this.sceneCache = {};
  this.width = 300;
  this.height = 150;
  this.scene = new THREE.Scene();
  this.renderer = new THREE.WebGLRenderer();
  //this.renderer = new THREE.WebGLRenderer({
  //  preserveDrawingBuffer   : true   // required to support .toDataURL()
  //});  
  this.loader = new THREE.ObjectLoader();
  this.sceneArea = dom_sceneArea;
  //renderer.setClearColor(0x88888888);
  this.renderer.setClearColor(0x888888FF);
  this.renderer.shadowMap.enabled = true;
  // to antialias the shadow
  this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;


  while (dom_sceneArea.firstChild) {
      dom_sceneArea.removeChild(dom_sceneArea.firstChild);
  }
  dom_sceneArea.appendChild( this.renderer.domElement );  //attach the three.js renderer to the proper div 

  this.cameraCallback = null;
  this.reset_camera();
  this.reset_scene();

  window.addEventListener('resize',function() { if(_this.sceneArea.clientWidth != 0) { _this.resize(_this.sceneArea.clientWidth,_this.sceneArea.clientHeight);}  });
  var initBind = function() {
    if(_this.sceneArea.clientWidth != 0 || _this.sceneArea.offsetWidth != 0) {
      console.log("KLAMPT: Resizing initial screen to "+_this.sceneArea.clientWidth+" x "+_this.sceneArea.clientHeight);
      _this.resize(_this.sceneArea.clientWidth,_this.sceneArea.clientHeight);
      _this.controls.update();
      _this.render();
    }
    else {
      setTimeout(initBind,10);
    }
  }
  setTimeout(initBind,0);
  
  return this;
} //end KlamptFrontend constructor

  var KLAMPT = { KlamptFrontend: KlamptFrontend };
  return KLAMPT;
});