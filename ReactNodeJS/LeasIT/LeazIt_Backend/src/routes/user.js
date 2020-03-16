"use strict";

const express  = require('express');
const router   = express.Router();

const middlewares    = require('../middlewares');
const UserController = require('../controllers/user');
const ItemController = require('../controllers/item');
const BookingController = require('../controllers/booking');
const Role           = require('../role_constants');

// get all users only permitted if ADMIN
router.get('/', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN), UserController.view_all_users);
// get my profile
router.get('/profile', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_LESSEE), UserController.get_profile);
// get another user's profile
router.get('/:id', UserController.view_user_profile);
router.get('/getByUsername/:username', UserController.getUserByUsername);
// edit my profile
router.put('/profile/edit', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_LESSEE), UserController.update_profile);
// delete my profile
router.delete('/profile/delete', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_LESSEE), UserController.delete_profile);
// delete a user permitted only if ADMIN
router.delete('/:id', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN), UserController.remove_user);
// request to become a lessor
router.put('/profile/request', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_LESSEE), UserController.update_profile);
// get all users with lessor requests if Admin
router.get('/admin/lessor_requests', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN), UserController.view_all_lessor_request);
// upgrade a user to lessor
router.put('/admin/lessor_requests/:id', middlewares.checkAuthentication, middlewares.roleAuthorization(Role.ROLE_ADMIN), UserController.grant_lessor_privileges);

module.exports = router;