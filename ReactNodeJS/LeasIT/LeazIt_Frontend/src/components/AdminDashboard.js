"use strict";

import React from 'react';
// import { DataTable, TableHeader, TableBody, TableRow, TableColumn, Button, Row, Col, Card, CardBody } from 'react-md';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';

import { Link } from 'react-router-dom';

import { SimpleLink } from './SimpleLink';

import AuthService from '../services/AuthService';
import Page from "./Page";

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




export class AdminDashboard extends React.Component {

    constructor(props) {
        super(props);

        this.onClick = this.onClick.bind(this);
    }
    onClick(value){
        this.props.data.history.push(value);
    }

    render() {
        return (

        <Page>
            <div className="animated fadeIn">
                <Row>
                    <h4>Welcome Admin</h4>
                    <div className="chart-wrapper mx-3" style={{ height: '70px' }}/>
                </Row>
                <Row>

                    <Col xs="12" sm="6" lg="3">
                        <Card className="text-white bg-info border border-dark">
                            <CardBody className="pb-0" onClick={() => this.onClick('/users')}>
                                <div className="text-center" >
                                    <FontAwesomeIcon icon="users" className={'fa-3x'}/>
                                </div>
                                <div className="chart-wrapper mx-3" style={{ height: '25px' }}/>
                                <div className="text-center"><h6>View all LeazIt users</h6></div>
                                <div className="chart-wrapper mx-3" style={{ height: '35px' }}/>
                            </CardBody>
                        </Card>
                    </Col>

                    <Col xs="12" sm="6" lg="3">
                        <Card className="text-white bg-info border border-dark">
                            <CardBody className="pb-0" onClick={() => this.onClick('/admin/view_lessor_requests')}>
                                <div className="text-center">
                                    <FontAwesomeIcon icon="file-upload" className={'fa-3x'}/>
                                    </div>
                                <div className="chart-wrapper mx-3" style={{ height: '25px' }}/>
                                <div className="text-center"><h6>View lessor requests</h6></div>
                                <div className="chart-wrapper mx-3" style={{ height: '35px' }}/>
                            </CardBody>
                        </Card>
                    </Col>

                    <Col xs="12" sm="6" lg="3">
                        <Card className="text-white bg-info border border-dark">
                            <CardBody className="pb-0" onClick={() => this.onClick('/add_category')}>
                                <div className="text-center">
                                    <FontAwesomeIcon icon="clipboard-list" className={'fa-3x'}/>
                                </div>
                                <div className="chart-wrapper mx-3" style={{ height: '25px' }}/>
                                <div className="text-center"><h6>View/add categories</h6></div>
                                <div className="chart-wrapper mx-3" style={{ height: '35px' }}/>
                            </CardBody>
                        </Card>
                    </Col>

                    <Col xs="12" sm="6" lg="3">
                        <Card className="text-white bg-info border border-dark" onClick={() => this.onClick('/my_conversations')}>
                            <CardBody className="pb-0">
                                <div className="text-center">
                                    <FontAwesomeIcon icon="envelope" className={'fa-3x'}/>
                                </div>
                                <div className="chart-wrapper mx-3" style={{ height: '25px' }}/>
                                <div className="text-center"><h6>View Messages</h6></div>
                                <div className="chart-wrapper mx-3" style={{ height: '35px' }}/>

                            </CardBody>
                        </Card>
                    </Col>
                </Row>
            </div>
        </Page>


        );
    }
}

// {/*<Page>*/}
// {/*<DataTable plain>*/}
// {/*<TableBody>*/}
// {/*<TableRow><SimpleLink to={'/users/'}>View all users</SimpleLink></TableRow>*/}
// {/*<TableRow><SimpleLink to={'/add_category/'}>Add a category</SimpleLink></TableRow>*/}
// {/*<TableRow><SimpleLink to={'/admin/view_lessor_requests/'}>View all Lessor Requests</SimpleLink></TableRow>*/}
// {/*<TableRow><SimpleLink to={'/my_conversations/'}>View my chats</SimpleLink></TableRow>*/}
// {/*</TableBody>*/}
// {/*</DataTable>*/}
// {/*<h2>Geia sou ksaderfe!</h2>*/}
//
// {/*</Page>*/}