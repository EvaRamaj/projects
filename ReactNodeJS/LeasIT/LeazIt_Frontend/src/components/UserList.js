"use strict";

import React from 'react';
import { FontIcon } from 'react-md';

import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Link } from 'react-router-dom';
import Page from './Page';

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


export const UserList = ({data,req, onDelete}) => (

    <Page>
        <div className="animated fadeIn">
            <Row >
                <Col className={'md'}>
                    <Card >
                        {/*<CardHeader>*/}
                            {/*LeazIt Users*/}
                        {/*</CardHeader>*/}
                        <CardBody>

                            <Table hover responsive className="table-outline mb-0 d-none d-table">
                                <thead className="thead-light">
                                <tr>
                                    <th className="text-center"><FontAwesomeIcon icon="users" /></th>
                                    <th>User</th>
                                    <th className="text-center"> <FontAwesomeIcon icon="user-cog" /><span>Account Type</span></th>
                                    <th className="text-center"> <FontAwesomeIcon icon="trash-alt" /><span>Delete User</span></th>

                                </tr>
                                </thead>

                                <tbody>
                                {Object.keys(data).map(function(key) {
                                    if(data[key].role !== 'Admin'){
                                    return(

                                            <tr>
                                                <td className="text-center">
                                                    <div className="avatar">
                                                        <Link to={`/profile/${data[key]._id}`}>
                                                        <img src= {`http://localhost:3000/photos/${data[key].photo}`} className="message-photo" alt={data[key].email}/>
                                                    </Link>
                                                    </div>
                                                </td>
                                                <td>
                                                    <div>
                                                        <Link to={`/profile/${data[key]._id}`}>{data[key].username}</Link>
                                                    </div>
                                                    <div className="small text-muted">
                                                        <span>{data[key].first_name}</span>|        {data[key].last_name}
                                                    </div>
                                                </td>
                                                <td className="text-center">
                                                    <span>{data[key].role}</span>
                                                </td>
                                                <td className="text-center">
                                                    <a className="btn btn-danger" href="#/users" onClick={(id) => onDelete(data[key]._id)}><FontAwesomeIcon icon="trash-alt"/></a>
                                                </td>
                                            </tr>

                                    )}})}

                                </tbody>
                            </Table>
                        </CardBody>
                    </Card>
                </Col>
            </Row>
        </div>
    </Page>

);

// {/*<Page>*/}
// {/*<DataTable plain>*/}
// {/*<TableHeader>*/}
// {/*<TableRow>*/}
// {/*<TableColumn></TableColumn>*/}
// {/*<TableColumn>Name</TableColumn>*/}
// {/*<TableColumn>Remove</TableColumn>*/}
// {/*<TableColumn>Approve?</TableColumn>*/}
// {/*</TableRow>*/}
// {/*</TableHeader>*/}
// {/*<TableBody>*/}
// {/*{data.map((user, i) => <UserListRow key={i} lessor_req={req} user={user} onDelete={(id) => onDelete(id)} />)}*/}
// {/*</TableBody>*/}
// {/*</DataTable>*/}
// {/*</Page>*/}