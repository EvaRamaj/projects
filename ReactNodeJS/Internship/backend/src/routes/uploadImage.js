"use strict";
const express  = require("express");
const multer  = require('multer');
const router   = express.Router();

const storage=multer.diskStorage({
    destination:function(req,file,cb){
        cb(null,'./uploads/');
    },
    filename:function(req,file,cb){
        cb(null,new Date().toISOString()+file.originalname);
    }
});
const fileFilter=(req,file,cb)=>{
    if(file.mimetype=='image/jpeg' || file.mimetype=='image/png' ){
        cb(null,true);
    }else{
        cb(null,false);
    }

    cb(null,false);
    cb(null,true);
};

const upload=multer({storage:storage, limits:{
        fileSize:1024*1024*5
    },
    fileFilter:fileFilter
});

router.post('/',upload.single('profileImage'),function(req,res){
    console.log('backend');
    console.log(req.file);
    let file = req.file;
    //res.sendStatus(200).send(file);
    res.send(file);
});

module.exports = router;