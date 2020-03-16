"use strict";

import React from 'react';
import { DataTable, TableHeader, TableBody, TableRow, TableColumn } from 'react-md';
import { Link } from 'react-router-dom';
import  MapContainer  from "./Map";
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import StarRatingComponent from "react-star-rating-component";
import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';
import Page from "./Page";
import { ToastContainer, toast } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';

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
    FormText,
    FormGroup,
    Form,
    Row,
    Table,
} from 'reactstrap';
import axios from "axios/index";

export class UserDashboard extends React.Component {

    constructor(props) {
        super(props);
        console.log('props:', props);
        this.state = {
            user: props.user,
            fileNames: [],
            selectedFile0: undefined,
        };
        this.notification = false;

        this.fileNames = [];
        this.handleChangeFile0 = this.handleChangeFile0.bind(this);
        this.handleSubmit = this.handleSubmit.bind(this);
        this.notify = this.notify.bind(this);
    }


    notify() {
        toast.error("User deleted", {
            position: toast.POSITION.TOP_CENTER
        });
        this.notification = true;
        let prop = this.props;
        setTimeout(function() {
            prop.onDelete()
        }, 1000);

    }
    handleChangeFile0 (event){
        this.setState(Object.assign({}, this.state, {selectedFile0: event.target.files[0]}));
    };

    handleSubmit(event) {
        event.preventDefault();
        let user = this.props.user;
        if(user === undefined) {
            user = {};
        }
        console.log(this.state.selectedFile0);
        if(this.state.selectedFile0 !== undefined) {
            const formData = new FormData();
            formData.append('file', this.state.selectedFile0);
            axios.post('http://localhost:3000/files', formData).then(
                response => {
                    this.fileNames.push(response.data.file.filename);
                    this.setState(Object.assign({}, this.state, {fileNames: this.fileNames}));
                    user.id_document = this.fileNames;
                    user.lessor_role_requested = true;
                    this.props.onSubmit(user);
                }
            )
        }
    }

    render() {
        if(this.state.loading){
            return(
                <h2>Loading....</h2>
            );
        }

        if(AuthService.getRole() === 'Lessee' && !this.state.user.lessor_role_requested){
            return (
                <Page>
                    <Row>
                        <Col xs="12" sm="12">
                            <h4 className='text-muted'>Welcome {this.props.user.username}</h4>
                            <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col xs="12" md="4">
                            <Card>
                                <CardBody>
                                    <tbody>
                                    <tr>
                                        <td>
                                            <div className="fakeimg text-center">
                                                <a>
                                                    <img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar" className="avatar" />
                                                    <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                                                </a>
                                            </div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <h5 className="text-center">{this.props.user.username}</h5>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <div className="text-center text-muted">{this.props.user.address}</div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td className="text-center">
                                            <StarRatingComponent
                                                name="rate1"
                                                starCount={10}
                                                value={this.props.evaluation}
                                            />
                                        </td>
                                    </tr>
                                    <tr>
                                        <td className="text-center">
                                            {/*<div className="chart-wrapper mx-3" style={{ height: '21px' }}/>*/}
                                        </td>
                                    </tr>
                                    </tbody>
                                </CardBody>
                            </Card>
                            <div className="chart-wrapper mx-3" style={{ height: '20px' }}/>
                            <Card>
                                <CardBody>
                                    <tr>
                                        <td>
                                            <SimpleLink to={'/my_conversations'} className="text-muted"><FontAwesomeIcon icon="envelope" /><span>View my conversations</span></SimpleLink>
                                        </td>
                                    </tr>
                                </CardBody>
                            </Card>
                        </Col>
                            <Col xs="12" md="4">
                                <Card >
                                    <CardBody>
                                        <Table hover responsive className="table-outline mb-1 d-none d-table">
                                            <thead className="thead-light">
                                            <tr>
                                                <th>Options</th>
                                            </tr></thead>
                                            <tbody>
                                            {/*<tr>*/}
                                                {/*<td>*/}
                                                    {/*<SimpleLink to={'/become_lessor'} className="text-muted"><FontAwesomeIcon icon="file-upload" /><span>Become a Lessor</span></SimpleLink>*/}
                                                {/*</td>*/}
                                            {/*</tr>*/}

                                            <tr>
                                                <td>
                                                    <SimpleLink to={'/my_bookings'} className="text-muted"><FontAwesomeIcon icon="shopping-cart" /><span>View my bookings</span></SimpleLink>
                                                </td>
                                            </tr>
                                            {/*<tr>*/}
                                                {/*<td>*/}
                                                    {/*<SimpleLink to={'/user_evaluations'} className="text-muted"><FontAwesomeIcon icon="star" /><span>View my evaluations</span></SimpleLink>*/}
                                                {/*</td>*/}
                                            {/*</tr>*/}
                                            <tr>
                                                <td href="/my_profile">
                                                    <SimpleLink to={'/my_profile'} className="text-muted"> <FontAwesomeIcon icon="cog" /><span>Profile Settings</span></SimpleLink>
                                                </td>
                                            </tr>
                                            <tr>
                                                <td onClick={this.notify} disabled={this.notification}>
                                                    <div className="text-muted" > <FontAwesomeIcon icon="trash-alt" /><span>Delete profile</span></div>
                                                </td>
                                            </tr>
                                            <ToastContainer autoClose={1000}/>
                                            </tbody>
                                        </Table>
                                    </CardBody>
                                </Card>
                            </Col>

                            <Col xs="12" lg="4">
                                <Card>
                                    <CardHeader>
                                        <strong>Do you want to offer your items, too?</strong>
                                    </CardHeader>
                                    <CardBody>
                                        <Form onSubmit={this.handleSubmit} className="form-horizontal">
                                            <FormGroup row>
                                                <Col xs="12" md="12">
                                                    <FormText className="help-block"><span>Please upload your id document</span></FormText>
                                                    <input type="file" id="idImage" placeholder="ID" onChange={this.handleChangeFile0}/>
                                                </Col>
                                            </FormGroup>

                                            <Row>
                                                <Col md="12" >
                                                    <Button id="submit" type="submit" size='sm' color="success" raised><i className="fa fa-upload"/> Upload</Button>
                                                </Col>
                                            </Row>
                                        </Form>
                                    </CardBody>
                                </Card>
                                {/*<MapContainer google={this.props.google} coord={this.props.user.coordinates}/>*/}
                            </Col>
                    </Row>
                </Page>
            );
        }
        else if (AuthService.getRole()=== 'Lessee' && this.state.user.lessor_role_requested) {
            return (
                <Page>
                    <Row>
                        <Col xs="12" sm="12">
                            <h4 className='text-muted'>Welcome {this.props.user.username}</h4>
                            <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col xs="12" md="4">
                            <Card>
                                <CardBody>
                                    <tbody>
                                    <tr>
                                        <td>
                                            <div className="fakeimg text-center">
                                                <a>
                                                    <img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar" className="avatar" />
                                                    <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                                                </a>
                                            </div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <h5 className="text-center">{this.props.user.username}</h5>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <div className="text-center text-muted">{this.props.user.address}</div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td className="text-center">
                                            <StarRatingComponent
                                                name="rate1"
                                                starCount={10}
                                                value={this.props.evaluation}
                                            />
                                        </td>
                                    </tr>
                                    <tr>
                                        <td className="text-center">
                                            {/*<div className="chart-wrapper mx-3" style={{ height: '21px' }}/>*/}
                                        </td>
                                    </tr>
                                    </tbody>
                                </CardBody>
                            </Card>
                            <div className="chart-wrapper mx-3" style={{ height: '20px' }}/>
                            <Card>
                                <CardBody>
                                    <tr>
                                        <td>
                                            <SimpleLink to={'/my_conversations'} className="text-muted"><FontAwesomeIcon icon="envelope" /><span>View my conversations</span></SimpleLink>
                                        </td>
                                    </tr>
                                </CardBody>
                            </Card>
                        </Col>
                        <Col xs="12" md="4">
                            <Card >
                                <CardBody>
                                    <Table hover responsive className="table-outline mb-1 d-none d-table">
                                        <thead className="thead-light">
                                        <tr>
                                            <th>Options</th>
                                        </tr></thead>
                                        <tbody>
                                        {/*<tr>*/}
                                            {/*<td>*/}
                                                {/*<SimpleLink to={'/become_lessor'} className="text-muted"><FontAwesomeIcon icon="file-upload" /><span>Become a Lessor</span></SimpleLink>*/}
                                            {/*</td>*/}
                                        {/*</tr>*/}

                                        <tr>
                                            <td>
                                                <SimpleLink to={'/my_bookings'} className="text-muted"><FontAwesomeIcon icon="shopping-cart" /><span>View my bookings</span></SimpleLink>
                                            </td>
                                        </tr>
                                        {/*<tr>*/}
                                            {/*<td>*/}
                                                {/*<SimpleLink to={'/user_evaluations'} className="text-muted"><FontAwesomeIcon icon="star" /><span>View my evaluations</span></SimpleLink>*/}
                                            {/*</td>*/}
                                        {/*</tr>*/}
                                        <tr>
                                            <td href="/my_profile">
                                                <SimpleLink to={'/my_profile'} className="text-muted"> <FontAwesomeIcon icon="cog" /><span>Profile Settings</span></SimpleLink>
                                            </td>
                                        </tr>
                                        <tr>
                                            <td onClick={this.notify} disabled={this.notification}>
                                                <div className="text-muted" > <FontAwesomeIcon icon="trash-alt" /><span>Delete profile</span></div>
                                            </td>
                                        </tr>
                                        <ToastContainer autoClose={1000}/>
                                        </tbody>
                                    </Table>
                                </CardBody>
                            </Card>
                        </Col>

                        <Col xs="12" lg="4">
                            <Card>
                                <CardHeader>
                                    <strong>Do you want to offer your items, too?</strong>
                                </CardHeader>
                                <CardBody>
                                    <Form onSubmit={this.handleSubmit} className="form-horizontal">
                                        <FormGroup row>
                                            <Col xs="12" lg="12">
                                                {/*<FormText className="help-block"><span>Request status</span></FormText><div/>*/}
                                                {/*<input type="file" id="idImage" placeholder="ID" onChange={this.handleChangeFile0}/>*/}
                                                {/*<div>Please be patient your request is being processed</div>*/}
                                                <h5><Badge className="lg" color={'warning'}>Please be patient as your id is being checked</Badge></h5>
                                            </Col>
                                        </FormGroup>

                                        {/*<Row>*/}
                                            {/*<Col md="12" >*/}
                                                {/*<Button disabled={this.state.user.lessor_role_requested} id="submit" type="submit" size='sm' color="success" raised><i className="fa fa-upload"/> Upload</Button>*/}
                                            {/*</Col>*/}
                                        {/*</Row>*/}
                                    </Form>
                                </CardBody>
                            </Card>
                            {/*<MapContainer google={this.props.google} coord={this.props.user.coordinates}/>*/}
                        </Col>
                    </Row>
                </Page>
            );
        }
        else if (AuthService.getRole()=== 'Lessor'){
            return (
                <Page>
                    <Row>
                        <Col xs="12" sm="12">
                            <h4 className='text-muted'>Welcome {this.props.user.username}</h4>
                            <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                        </Col>
                    </Row>
                    <Row>
                        <Col xs="12" md="4">
                            <Card>
                                <CardBody>
                                    <tbody>
                                    <tr>
                                        <td>
                                            <div className="fakeimg text-center">
                                                <a>
                                                    <img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt="Avatar" className="avatar" />
                                                    <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                                                </a>
                                            </div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <h5 className="text-center">{this.props.user.username}</h5>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td>
                                            <div className="text-center text-muted">{this.props.user.address}</div>
                                        </td>
                                    </tr>
                                    <tr>
                                        <td className="text-center">
                                            <StarRatingComponent
                                                name="rate1"
                                                starCount={10}
                                                value={this.props.evaluation}/>
                                        </td>
                                    </tr>
                                    </tbody>
                                </CardBody>
                            </Card>
                            <div className="chart-wrapper mx-3" style={{ height: '20px' }}/>
                            <Card>
                                <CardBody>
                                    <tr>
                                        <td>
                                            <SimpleLink to={'/my_conversations'} className="text-muted"><FontAwesomeIcon icon="envelope" /><span>View my conversations</span></SimpleLink>
                                        </td>
                                    </tr>
                                </CardBody>
                            </Card>
                        </Col>
                        <Col xs="12" md="4">
                            <Card >
                                <CardBody>
                                    <Table hover responsive className="table-outline mb-1 d-none d-table">
                                        <thead className="thead-light">
                                        <tr>
                                            <th>Options</th>
                                        </tr></thead>
                                        <tbody>
                                        <tr>
                                            <td>
                                                <SimpleLink to={'/my_profile'} className="text-muted"> <FontAwesomeIcon icon="cog" /><span>Profile Settings</span></SimpleLink>
                                            </td>
                                        </tr>

                                        <tr>
                                            <td>
                                                <SimpleLink to={'/my_bookings'} className="text-muted"><FontAwesomeIcon icon="shopping-cart" /><span>View my bookings</span></SimpleLink>
                                            </td>
                                        </tr>
                                        {/*<tr>*/}
                                            {/*<td>*/}
                                                {/*<SimpleLink to={'/user_evaluations'} className="text-muted"><FontAwesomeIcon icon="star" /><span>View my evaluations</span></SimpleLink>*/}
                                            {/*</td>*/}
                                        {/*</tr>*/}
                                        <tr>
                                            <td>
                                                <SimpleLink to={'/add'} className="text-muted"> <FontAwesomeIcon icon="plus-square" /><span>Add new item</span></SimpleLink>
                                            </td>
                                        </tr>
                                        <tr>
                                            <td>
                                                <SimpleLink to={'/my_items'} className="text-muted"> <FontAwesomeIcon icon="clipboard-list" /><span>View my Items</span></SimpleLink>
                                            </td>
                                        </tr>

                                        <tr>
                                            <td onClick={this.notify} disabled={this.notification}>
                                                <div className="text-muted" > <FontAwesomeIcon icon="trash-alt" /><span>Delete profile</span></div>
                                            </td>
                                        </tr>
                                        <ToastContainer autoClose={1000}/>
                                        </tbody>
                                    </Table>
                                </CardBody>
                            </Card>
                        </Col>
                        <Col xs="12" lg="4">
                            <MapContainer google={this.props.google} coord={this.props.user.coordinates}/>

                            {/*<Card >*/}
                            {/*<CardHeader>*/}
                            {/*Current Location*/}
                            {/*</CardHeader>*/}
                            {/*<CardBody>*/}
                            {/*<MapContainer google={this.props.google} coord={this.props.user.coordinates}/>*/}
                            {/*</CardBody>*/}
                            {/*</Card>*/}
                        </Col>
                    </Row>
                        {/*<DataTable plain>*/}
                        {/*<TableBody>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/my_bookings'}>View my bookings</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/my_conversations'}>View my conversations</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/add'}>Add Item</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/user_evaluations'}>View my evaluations</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/my_items'}>View my items</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/my_profile'}>View my profile</SimpleLink></TableColumn>*/}
                            {/*</TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn><SimpleLink to={'/edit_profile/'}>Edit my profile</SimpleLink></TableColumn></TableRow>*/}
                            {/*<TableRow>*/}
                                {/*<TableColumn>*/}
                                    {/*<Button disabled={this.notification} onClick={this.notify} raised>Delete my profile</Button>*/}
                                    {/*<ToastContainer autoClose={1000}/>*/}
                                {/*</TableColumn>*/}
                            {/*</TableRow>*/}


                        {/*</TableBody>*/}
                    {/*</DataTable>*/}
                    {/*<h2>Geia sou ksaderfe!</h2>*/}

                </Page>
            );

        }
    }
}