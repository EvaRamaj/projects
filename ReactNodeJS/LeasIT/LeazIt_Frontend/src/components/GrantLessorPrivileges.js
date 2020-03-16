"use strict";

import React from 'react';
import { Link } from 'react-router-dom'
import { Media, MediaOverlay, Grid, Cell, FontIcon } from 'react-md';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import {
    Badge,
    Button,
    ButtonDropdown,
    ButtonGroup,
    ButtonToolbar,
    Card,
    CardBody,
    CardFooter,
    CardHeader,
    CardTitle,
    Col,
    Dropdown,
    DropdownItem,
    DropdownMenu,
    DropdownToggle,
    Progress,
    Row,
    Table,
} from 'reactstrap';

import Page from './Page';
import UserService from "../services/UserService";

export class GrantLessorPrivileges extends React.Component {

    constructor(props) {
        super(props);
        console.log(props);

        if(this.props.user != undefined) {
            this.state = {
                _id: props.user._id,
                username: props.user.username,
                password: props.user.password,
                email: props.user.email,
                first_name: props.user.first_name,
                last_name: props.user.last_name,
                address: props.user.address,
                phone: props.user.phone,
                photo: props.user.photo,
                id_document: props.user.id_document,
                lessor_role_requested: props.user.lessor_role_requested,
                role: props.user.role
            };
        }

        this.handleApproveUser = this.handleApproveUser.bind(this);
        this.handleDenyUser = this.handleDenyUser.bind(this);

    }


    handleApproveUser(){


        let user = this.state;
        user.is_Lessor = true;
        user.lessor_role_requested = false;
        user.role = 'Lessor';


        UserService.grantLessorPrivileges(user).then((data) => {
            this.props.history.push('/admin_dashboard');
        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while updating a user'}));
        });



    }



    handleDenyUser() {

        let user = this.state;
        user.lessor_role_requested = false;
        user.is_Lessor = false;

        UserService.grantLessorPrivileges(user).then((data) => {
            this.props.history.push('/admin_dashboard');

        }).catch((e) => {
            console.error(e);
            this.setState(Object.assign({}, this.state, {error: 'Error while updating a user'}));
        });

    }



    render() {
        return (
            <Page>
                <Row>
                    <Col xs="12" md="4">
                        <Card style={{ width: '300px' }}>
                            <CardHeader className="small text-muted"> User Identification Document</CardHeader>
                            <CardBody>
                                <img className="lessor_id" src={`http://localhost:3000/files/${this.props.user.id_document}`} alt={this.props.user.username}/>
                            </CardBody>
                        </Card>
                    </Col>
                    <Row>
                        <Col xs="12" md="8">
                            <Card style={{ width: '280px' }}>
                                <CardHeader className="small text-muted">Do you want to grant lessor privileges to: </CardHeader>
                                <CardBody>
                                    {/*<div className="small text-muted"></div>*/}
                                    <h6>{this.props.user.first_name}    {this.props.user.last_name}</h6>
                                    <a className="btn btn-success" href="#/admin/view_lessor_requests" onClick={() => this.handleApproveUser()}><FontAwesomeIcon icon="check"/> Accept</a>
                                    <span/>
                                    <a className="btn btn-danger" href="#/admin/view_lessor_requests" onClick={() => this.handleDenyUser()}><FontAwesomeIcon icon="times"/> Decline</a>
                                </CardBody>
                            </Card>
                        </Col>
                    </Row>
                </Row>

                {/*<div className="card" style={{ width: '400px' }}>*/}
                    {/*<img className="lessor_id" src={`http://localhost:3000/files/${this.props.user.id_document}`} alt={this.props.user.username}/>*/}
                        {/*<div className="card-body">*/}
                            {/*<h5 className="card-title">{this.props.user.username}</h5>*/}
                            {/*<p className="small text-muted">{this.props.user.first_name}|        {this.props.user.last_name}</p>*/}

                            {/*<a href="#" className="btn btn-primary">See Profile</a>*/}
                        {/*</div>*/}
                {/*</div>*/}
                {/*<Card>*/}
                    {/*<Grid className="grid-example" >*/}
                        {/*<Cell size={3}>*/}
                            {/*<Media aspectRatio="1-1">*/}
                                {/*<img src={`http://localhost:3000/files/${this.props.user.id_document}`} alt={this.props.user.username} />*/}
                            {/*</Media>*/}
                            {/*<button onClick={this.handleApproveUser} >*/}
                                {/*Approve User*/}
                            {/*</button>*/}
                            {/*<button onClick={this.handleDenyUser}>*/}
                                {/*Deny User*/}
                            {/*</button>*/}
                        {/*</Cell>*/}
                        {/*<Cell size={7}/>*/}
                    {/*</Grid>*/}
                {/*</Card>*/}
            </Page>
        );
    }
};