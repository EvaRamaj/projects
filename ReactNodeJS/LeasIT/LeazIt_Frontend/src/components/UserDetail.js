"use strict";

import React from 'react';
import { Link, withRouter } from 'react-router-dom';
import { SimpleLink } from './SimpleLink';

// import { Card, CardTitle, CardText, Media, MediaOverlay, Grid, Cell, Button, FontIcon } from 'react-md';
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
import Page from './Page';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import StarRatingComponent from "react-star-rating-component";
import { ToastContainer, toast } from 'react-toastify';

import AuthService from '../services/AuthService';
import  MapContainer  from "./Map";

class UserDetail extends React.Component {

    constructor(props) {
        console.log(props);
        super(props);
    }
    onEdit(){
        this.props.history.push('/edit_profile');
    }

    render() {
        let user = this.props.user;
        let self = this;
        // --TODO change in a way that booleans can be used
        return (

            <Page>
                <Row>
                    <Col xs="12" sm="12">
                        <h4 className='text-muted'>{this.props.user.username} details</h4>
                        <div className="chart-wrapper mx-3" style={{ height: '7px' }}/>
                    </Col>
                </Row>
                <Row>
                    <Col xs="12" md="4">
                        <Card>
                            <CardBody>
                                <tbody>
                                {/*{Object.keys(data).map(function(key) {*/}
                                    {/*if (data[key].role !== 'Admin') {*/}
                                        {/*return (*/}
                                        {/*);*/}
                                    {/*}*/}
                                {/*}*/}
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
                        <Row>
                            {Object.keys(user).map(function(key) {
                                    if(self.props.match.path !== "/profile/:id" && key === 'password'){
                                return(

                                        <Col md="12" md='12' className='text-center'>
                                            {/*<div style={{display: 'flex'}} className="btn btn-success" href="/edit_profile"><i className="fa fa-edit"/> Edit</div>*/}
                                            <Button className="text-center" onClick={() => self.onEdit()} color="success" raised><i className="fa fa-edit"/> Edit Profile</Button>
                                        </Col>
                                );
                                    }})}

                        </Row>
                    </Col>
                    <Col xs="12" md="5">
                        <Card >
                            <CardBody>
                                <Table className="table-outline mb-1 d-none d-table">
                                    <thead className="thead-light">
                                    <tr>
                                        <th>User details</th>
                                        <th/>
                                    </tr></thead>
                                    <tbody>

                                    {Object.keys(user).map(function(key) {
                                        if(key !== '_id' && key !=='__v' && key !=='photo' && key!=='password' &&
                                            key !=='coordinates' && key !== 'is_Lessor' && key !== 'id_document'
                                        && key !== 'lessor_role_requested' && key !== 'role') {
                                            let s = key.toString();
                                            let out = s.split("_");
                                            let handler = 'handleChange';
                                            let label = '';
                                            for (let w in out){
                                                handler += out[w].substring(0, 1).toUpperCase() + out[w].substring(1);
                                                label +=' ' + out[w].substring(0, 1).toUpperCase() + out[w].substring(1);
                                            }
                                            return (
                                                <tr>
                                                    <td>
                                                        <div className="text-muted">{label}:</div>
                                                    </td>
                                                    <td>
                                                        {user[key]}
                                                    </td>
                                                </tr>
                                            );
                                        }

                                            }
                                        )
                                    }


                                    <ToastContainer autoClose={1000}/>
                                    </tbody>
                                </Table>
                            </CardBody>
                        </Card>
                    </Col>
                    <Col xs="12" md="3">
                        <Row>
                            <MapContainer google={this.props.google} coord={user.coordinates}/>
                        </Row>
                    </Col>
                </Row>
            </Page>
            // <Page>
            //     <Card>
            //         <Grid className="grid-example" >
            //             <Cell size={3}>
            //                 <Media aspectRatio="1-1">
            //                     <img src={`http://localhost:3000/photos/${this.props.user.photo}`} alt={this.props.user.username} />
            //                 </Media>
            //             </Cell>
            //             <Cell size={7}/>
            //         </Grid>
            //
            //         <CardTitle title={this.props.user.username} subtitle={this.props.user.email} />
            //
            //         <CardText>
            //             {Object.keys(user).map(function(key) {
            //                 if(key !== '_id' && key !=='__v' && key !=='photo' && key!=='password' && key !=='coordinates' && key !== 'is_Lessor'){
            //
            //                         if((key === 'is_Lessor' || key === 'lessor_role_requested') &&  user[key] === true){
            //
            //                         return <div>{key}: True</div>
            //                     }
            //                     else if ((key === 'is_Lessor' || key === 'lessor_role_requested') &&  user[key] === false){
            //                         return <div>{key}: False</div>
            //                     }
            //                     else{
            //                         return <div>{key}: {user[key]}</div>
            //                     }
            //
            //                 }
            //
            //             })}
            //         </CardText>
            //         <button onClick={() => this.onEdit()} raised>Edit</button>
            //
            //     </Card>
            //     <MapContainer google={this.props.google} coord={user.coordinates}/>
            //
            // </Page>

        );
    }
}
export default withRouter(UserDetail);