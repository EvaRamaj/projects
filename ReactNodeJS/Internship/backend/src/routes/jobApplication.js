"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const ApplicationController = require('../controllers/jobApplication');


//router.get('/', JobController.listAll); // list all jobs
router.post('/create', ApplicationController.create); // create new job
router.get('/myApplications', ApplicationController.getMyApplications); // get all the job applications per owner

// router.post('/doComment', middlewares.checkAuthentication, BlogController.insertComment); // add comment to blog
// router.get('/featured', BlogController.getFeaturedBlog); // Read featured blog
router.get('/:id', ApplicationController.read); // Read a job by Id
router.put('/:id', ApplicationController.update); // Update a blog by Id
// router.delete('/:id', middlewares.checkAuthentication, BlogController.remove); // Delete a movie by Id

module.exports = router;
