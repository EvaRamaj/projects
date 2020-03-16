"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const CategoryController = require('../controllers/category');
const Role           = require('../role_constants');


router.get('/',CategoryController.list); // List all movies
router.post('/', middlewares.checkAuthentication,middlewares.roleAuthorization(Role.ROLE_ADMIN),CategoryController.create); // Create a new movie
router.get('/:id', CategoryController.read); // Read a movie by Id
router.put('/:id', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN),CategoryController.update); // Update a movie by Id
router.delete('/:id', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN),CategoryController.remove); // Delete a movie by Id


module.exports = router;