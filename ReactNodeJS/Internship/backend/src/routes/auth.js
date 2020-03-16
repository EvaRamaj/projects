"use strict";

const express        = require('express');
const router         = express.Router();

const AuthController = require('../controllers/auth');

router.post('/login', AuthController.login);
router.post('/register', AuthController.register);
router.post('/linkedinLogin', AuthController.linkedinAccessToken, AuthController.linkedInLogin);
router.post('/linkedinSignup', AuthController.linkedinAccessToken, AuthController.linkedInSignup);
router.put('/confirmEmail', AuthController.confirmEmail);

module.exports = router;