define(function(){

    function load_ipython_extension(){
        console.info('Loading the klampt-jupyter-widget extension');
        //older version uses jupyter-js-widgets, not @jupyter-widgets/base
        define('klampt-jupyter-widget', ["@jupyter-widgets/base","nbextensions/klampt/KlamptFrontend"], function(widgets,KLAMPT) {
            if(widgets===undefined) {
                throw new Error("jupyter-widgets/base was not found")
            }
            if(KLAMPT===undefined) {
                throw new Error("KlamptFrontend.js was not found")
            }
            var KlamptModel = widgets.DOMWidgetModel.extend({
                defaults: _.extend(widgets.DOMWidgetModel.prototype.defaults(), {
                    _model_name : 'KlamptModel',
                    _view_name : 'KlamptView',
                    _model_module : 'klampt-jupyter-widget',
                    _view_module : 'klampt-jupyter-widget',
                    _model_module_version : '0.1.1',
                    _view_module_version : '0.1.1',
                    _camera : {'camera is':'first time in javascript'}
                })
            });

            function isEmpty(ob){
               for(var i in ob){ return false;}
              return true;
            }

            var KlamptView = widgets.DOMWidgetView.extend({
                initialize: function(parameters) {
                    console.log("KlamptView.initialize "+parameters);
                    this.klampt = null;
                    this.waitDraw = false;
                    widgets.DOMWidgetView.prototype.initialize.call(this, parameters);
                },
                createDiv: function(){
                    var width = this.model.get('width');
                    var height = this.model.get('height');
                    var div = document.createElement("div");
                    div.style.width = width+"px";
                    div.style.height = height+"px";
                    return div;
                    },
                // Render the view.
                render: function() {
                    console.log("Creating DOM element for Klampt frontend");
                    this.klamptArea = this.createDiv();
                    this.el.appendChild(this.klamptArea);
                    this.klampt = new KLAMPT.KlamptFrontend(this.klamptArea);
                    var _this = this;
                    this.klampt.set_camera_callback(function() {
                            var cam = _this.klampt.get_camera();
                            _this.model.set('_camera',cam);
                            _this.touch();
                        });
                    this.messageArea = document.createElement("div");
                    this.klampt.renderer.domElement.addEventListener( 'keydown', this.keydown, false );
                    this.el.appendChild(this.messageArea);
                    this.model.on('change:scene', this.scene_changed, this);
                    this.model.on('change:transforms', this.transforms_changed, this);
                    this.model.on('change:rpc', this.rpc_changed, this);
                    //this.klampt.resize(this.model.get('width'),this.model.get('height'));

                    //write initial camera to model
                    var cam = this.klampt.get_camera();
                    this.model.set('_camera',cam);
                    var redraw = false;
                    if(!isEmpty(this.model.get('scene'))) {
                        console.log("KlamptView.render: scene not empty");
                        //this.scene_changed();
                        this.klampt.update_scene(this.model.get('scene'));
                        redraw=true;
                    }
                    if(!isEmpty(this.model.get('transforms'))) {
                        console.log("KlamptView.render: transforms not empty");
                        //this.transforms_changed();
                        this.klampt.update_scene(this.model.get('transforms'));
                        redraw=true;
                    }
                    if(!isEmpty(this.model.get('rpc'))) {
                        console.log("KlamptView.render: rpc not empty");
                        //this.rpc_changed();
                        this.do_rpc(this,this.model.get('rpc'));
                        redraw=true;
                    }
                    if(redraw) {
                        var _this=this;
                        setTimeout(function() { 
                            _this.klampt.render();
                            _this.model.set('drawn',1);
                            _this.touch();
                        },0);
                    }
                },
                keydown : function(event) {
                    console.log("Got a keydown event "+event.keyCode);
                    events = this.model.get('events').clone();
                    events.append(event)
                    this.model.set('event',events);
                    this.touch();
                },
                scene_changed: function() {
                    var msg = this.model.get('scene');
                    console.log("Klamp't widget: setting scene");
                    if(this.waitDraw) {
                        this.klampt.update_scene(msg);
                    }
                    else {
                        var _this = this;
                        this.waitDraw=true;
                        setTimeout(function() {
                            _this.klampt.update_scene(msg);
                            _this.klampt.render();
                            _this.model.set('drawn',1);
                            _this.touch();
                            _this.waitDraw=false;
                            },0);
                    }
                },
                transforms_changed: function() {
                    var msg = this.model.get('transforms');
                    //console.log("Klamp't widget: setting transforms");
                    if(this.waitDraw) {
                        this.klampt.update_scene(msg);
                    }
                    else {
                        var _this = this;
                        this.waitDraw=true;
                        setTimeout(function() {
                            _this.klampt.update_scene(msg);
                            _this.klampt.render();
                            _this.model.set('drawn',1);
                            _this.touch();
                            _this.waitDraw=false;
                            },0);
                    }
                },
                do_rpc : function(_this,msg) {
                    if(msg.type == 'multiple') {
                        for(var i=0; i<msg.calls.length; i++) {
                            _this.do_rpc(_this,msg.calls[i]);
                        }
                    }
                    else if(msg.type == 'reset_scene') {
                        //console.log("Klamp't widget: resetting scene");
                        _this.klampt.reset_scene();
                    }
                    else if(msg.type == 'reset_camera') {
                        //console.log("Klamp't widget: calling reset_camera");
                        _this.klampt.reset_camera();
                    }
                    else {
                        //console.log("Klamp't widget: calling rpc "+msg);
                        _this.klampt.rpc(msg);
                    }
                },
                rpc_changed: function() {
                    var msg = this.model.get('rpc');
                    //console.log("rpc "+msg);
                    if(this.waitDraw) {
                        this.do_rpc(this,msg);
                    }
                    else {
                        var _this = this;
                        this.waitDraw=true;
                        setTimeout(function() { 
                            _this.do_rpc(_this,msg); 
                            _this.klampt.render();
                            _this.model.set('drawn',1);
                            _this.touch();
                            _this.waitDraw=false;
                        },0);
                    }
                }
                
            });

            return {
                KlamptModel: KlamptModel,
                KlamptView: KlamptView
            };
        });

    }

    return {
        load_ipython_extension: load_ipython_extension
    };
});
