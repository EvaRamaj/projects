"use strict";

import React from 'react';
import { FontIcon } from 'react-md';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
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

// function UserRow(props) {
//     const user = props.user
//     const userLink = `#/users/${user.id}`
//
//     const getBadge = (status) => {
//         return status === 'Active' ? 'success' :
//             status === 'Inactive' ? 'secondary' :
//                 status === 'Pending' ? 'warning' :
//                     status === 'Banned' ? 'danger' :
//                         'primary'
//     }

export const UserListLessor = ({data,req, onApprove, onDeny, onDetails}) => (
    <Page>
        <div className="animated fadeIn">
            <Row>
                <Col>
                    <Card>
                        {/*<CardHeader>*/}
                            {/*LeazIt Users*/}
                        {/*</CardHeader>*/}
                        <CardBody>

                            <Table hover responsive className="table-outline mb-0 d-none d-sm-table">
                                <thead className="thead-light">
                                <tr>
                                    <th className="text-center">
                                        <a className="btn btn-info" onClick={() => window.location.reload()} style={{color: 'white'}}><FontAwesomeIcon icon="sync-alt"/></a>
                                    </th>
                                    <th><FontAwesomeIcon icon="users" /><span>User</span></th>
                                    <th className="text-center"> <FontAwesomeIcon icon="user-cog" /><span>Account Type</span></th>

                                    <th className="text-center">Status</th>
                                    <th className="text-center"><span>Decision</span></th>
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
                                                            <img src= {`http://localhost:3000/photos/${data[key].photo}`}  className="message-photo" alt={data[key].email}/>
                                                        </Link>
                                                    </div>
                                                </td>
                                                <td>
                                                    <div>
                                                        <Link to={`/profile/${data[key]._id}`} style={{color: 'black'}}>{data[key].username}</Link>
                                                    </div>
                                                    <div className="small text-muted">
                                                        <span>{data[key].first_name}</span>|        {data[key].last_name}
                                                    </div>
                                                </td>
                                                <td className="text-center">
                                                    <span>{data[key].role}</span>
                                                </td>

                                                <td className="text-center"><Badge type='btn btn-default' onClick={() => onDetails(data[key]._id)} color={'warning'}>Request Pending</Badge></td>
                                                <td className="text-center">
                                                    <a className="btn btn-success" href="#/admin/view_lessor_requests" onClick={(id) => onApprove(data[key]._id)}><FontAwesomeIcon icon="check"/></a>
                                                    <span/>
                                                    <a className="btn btn-danger" href="#/admin/view_lessor_requests" onClick={(id) => onDeny(data[key]._id)}><FontAwesomeIcon icon="times"/></a>
                                                    {/*<span><button className=""><FontAwesomeIcon icon="check"color={'success'}/></button></span>*/}
                                                    {/*<span><FontIcon type='button' onClick={(id) => onDelete(data[key]._id)}>delete</FontIcon></span>*/}
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