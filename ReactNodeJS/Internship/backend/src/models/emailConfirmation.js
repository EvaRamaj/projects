const express  = require("express");
const router   = express.Router();
const bodyParser = require('body-parser');
const nodemailer = require('nodemailer');
const config     = require('../config');
const jwt        = require('jsonwebtoken');


router.use(bodyParser.urlencoded({ extended: false }));

router.get('/', function(req,res,next){
    //Get token
    var token=req.query.token;
    console.log(token);
    //decrypt token
    jwt.verify(token, config.JwtSecret, (err, decoded) => {
        if (err) return res.status(401).send({
            error: 'Unauthorized',
            message: 'Failed to authenticate token.'+err.message
        });

        console.log(decoded.id);
        console.log(decoded.username);
      
    //upadte database
    })
});

router.post('/',function(req,res){
    console.log('post');
    
});


module.exports = router;