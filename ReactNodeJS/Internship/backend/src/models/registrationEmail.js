const express  = require("express");
const router   = express.Router();
const bodyParser = require('body-parser');
const nodemailer = require('nodemailer');
const config     = require('config');
const configFile     = require('../config');
const jwt        = require('jsonwebtoken');

router.use(bodyParser.urlencoded({ extended: false }));

router.get('/', function(req,res,next){
    res.render('index',{title:'Express'});dest
})

router.post('/',function(req,res){
    
    const token = jwt.sign({ id: "1234", username: "rakyman",
     role: "admin"}, configFile.JwtSecret, {
        expiresIn: 86400 // expires in 24 hours
    });

    const output = `
    <p>Hello Mr/s ${req.body.name} <br/> <br/> 
    
    Please go and fill out your profile so that you get connected to <br/>
    the best matching project via the follow link: http://localhost:8080/confirmation?token=${token} 
    
    </p>
  `;


  let transporter = nodemailer.createTransport({
    service: 'gmail',
    auth: {
           user: config.get('App.emailCredentials.email'),
           pass: config.get('App.emailCredentials.password')
       }
   });

  // setup email data with unicode symbols
  let mailOptions = {
     from: config.get('App.emailCredentials.email'), // sender address
      to: req.body.email, // list of receivers
      subject: 'Message from the wayer platform', // Subject line
      text: 'Hello Mr/s '+req.body.name+'<br/>'+' Click here to continue with the sign up process', // plain text body
      html: output // html body
  };

  // send mail with defined transport object
  transporter.sendMail(mailOptions, (error, info) => {
      if (error) {
          return console.log(error);
      }
      console.log('Message sent: %s', info.messageId);   
      console.log('Preview URL: %s', nodemailer.getTestMessageUrl(info));

      res.render('contact', {msg:'Email has been sent'});
    }); 
});


module.exports = router;