"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const CompanyController = require('../controllers/company');


router.post('/search', CompanyController.search); // Search retired professionals
router.post('/sendMail', CompanyController.sendMail); // Search retired professionals
//router.delete('/:id', middlewares.checkAuthentication, MovieController.remove); // Delete a movie by Id

module.exports = router;
