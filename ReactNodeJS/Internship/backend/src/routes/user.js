"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const UserController = require('../controllers/user');



//router.get('/', MovieController.list); // List all movies
//router.post('/', UserController.create); // Create a new movie
router.get('/count', UserController.count); // Read a movie by Id
router.get('/:id', UserController.read); // Read a movie by Id
//router.get('/profile',UserController.get_profile);
router.put('/:id', middlewares.checkAuthentication, UserController.update_profile);

//router.put('/:id', middlewares.checkAuthentication, MovieController.update); // Update a movie by Id
//router.delete('/:id', middlewares.checkAuthentication, MovieController.remove); // Delete a movie by Id


module.exports = router;
